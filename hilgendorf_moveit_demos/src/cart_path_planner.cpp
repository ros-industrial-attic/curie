/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Creates a cartesian path to be inserted into a planning roadmap
*/

// this package
#include <hilgendorf_moveit_demos/cart_path_planner.h>
#include <hilgendorf_moveit_demos/hilgendorf_demos.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

namespace hilgendorf_moveit_demos
{

  CartPathPlanner::CartPathPlanner(HilgendorfDemos* parent)
    : name_("cart_path_planner")
    , nh_("~")
    , parent_(parent)
  {
    // Load rosparams
    //ros::NodeHandle rpnh(nh_, name_);
    //std::size_t error = 0;
    //error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    //rosparam_shortcuts::shutdownIfError(name_, error);

    // Load planning state
    imarker_state_.reset(new moveit::core::RobotState(*parent_->start_state_));

    // Load thread for user feedback
    imarker_thread_ = std::thread(&CartPathPlanner::imarkerThread, this);

    ROS_INFO_STREAM_NAMED(name_,"CartPathPlanner Ready.");
  }

  CartPathPlanner::~CartPathPlanner()
  {
    is_shutting_down_ = true;
    //std::cout << "Waiting to join interactive marker thread... " << std::endl;
    imarker_thread_.join();
  }

  void CartPathPlanner::imarkerThread()
  {
    Eigen::Affine3d pose;
    Eigen::Affine3d prev_pose;

    ros::Rate rate(30.0);
    while (!ros::isShuttingDown() && !is_shutting_down_)
    {
      rate.sleep();

      if (!parent_->getTFTransform("world", "right_ee", pose))
      {
        ROS_INFO_STREAM_NAMED(name_, "Waiting to recieve /tf for 'right_ee'");
        continue;
      }

      // Check if poses have changed value at all
      if (parent_->visual_start_->posesEqual(prev_pose, pose))
      {
        continue;
      }

      // Attempt to set robot to new pose
      ROS_DEBUG_STREAM_THROTTLE_NAMED(1, name_, "Setting from IK");
      if (imarker_state_->setFromIK(parent_->jmg_, pose))
      {
        parent_->visual_start_->publishRobotState(imarker_state_);

        // Save current pose
        prev_pose = pose;
        imarker_pose_changed_ = true;
        computePath();
      }
      else
        ROS_DEBUG_STREAM_NAMED(name_, "Failed to set IK");
    }
  }

  bool CartPathPlanner::computePath()
  {
    // Plan cartesian path
    std::vector<moveit::core::RobotStatePtr> traj;
    Eigen::Vector3d rotated_direction;
    rotated_direction << 0, -1, 0;
    double desired_distance = 0.5; // Distance to move
    double max_step = 0.01; // Resolution of trajectory, the maximum distance in Cartesian space between consecutive points on the resulting path

    // this is the Cartesian pose we start from, and we move in the direction indicated
    Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);
    traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*imarker_state_)));

    //The target pose is built by applying a translation to the start pose for the desired direction and distance
    Eigen::Affine3d target_pose = start_pose;
    target_pose.translation() += rotated_direction * desired_distance;

    // Create new state
    moveit::core::RobotState robot_state(*imarker_state_);
    // Decide how many steps we will need for this trajectory
    double distance = (target_pose.translation() - start_pose.translation()).norm();
    unsigned int steps = 5 + (unsigned int)floor(distance / max_step);

    std::vector<double> dist_vector;
    double total_dist = 0.0;
    double last_valid_percentage = 0.0;
    Eigen::Quaterniond start_quaternion(start_pose.rotation());
    Eigen::Quaterniond target_quaternion(target_pose.rotation());

    parent_->visual_start_->enableBatchPublishing();
    parent_->visual_start_->deleteAllMarkers();
    for (unsigned int i = 1; i <= steps ; ++i)
    {
      double percentage = (double)i / (double)steps;

      Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
      pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

      // Visualize
      parent_->visual_start_->publishArrow(pose, rvt::ORANGE, rvt::REGULAR, 0.05, i);

      if (robot_state.setFromIK(parent_->jmg_, pose, parent_->ee_link_->getName(), 1, 0.0))
      {
        traj.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_state)));

        // compute the distance to the previous point (infinity norm)
        double dist_prev_point = traj.back()->distance(*traj[traj.size() - 2], parent_->jmg_);
        dist_vector.push_back(dist_prev_point);
        total_dist += dist_prev_point;
      }
      else
        break;
      last_valid_percentage = percentage;
    }
    parent_->visual_start_->triggerBatchPublishAndDisable();

    //std::cout << "last_valid_percentage: " << last_valid_percentage << std::endl;

    if (!last_valid_percentage)
    {
      ROS_WARN_STREAM_NAMED(name_, "No solution found");
      return false;
    }

    // Visualize path
    // double speed = 0.001;
    // bool blocking = true;
    // parent_->visual_start_->publishTrajectoryPath(traj, parent_->jmg_, speed, blocking);

    imarker_pose_changed_ = false;
    return true;
  }

} // namespace hilgendorf_moveit_demos
