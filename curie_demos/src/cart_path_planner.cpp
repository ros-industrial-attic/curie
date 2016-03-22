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
#include <curie_demos/cart_path_planner.h>
#include <curie_demos/curie_demos.h>

// ROS parameter loading
//#include <rosparam_shortcuts/rosparam_shortcuts.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

namespace curie_demos
{
CartPathPlanner::CartPathPlanner(CurieDemos *parent) : name_("cart_path_planner"), nh_("~"), parent_(parent)
{
  // Load planning state
  imarker_state_.reset(new moveit::core::RobotState(*parent_->moveit_start_));

  // Create cartesian start pose interactive marker
  imarker_cartesian_.reset(new IMarkerRobotState(parent_->getPlanningSceneMonitor(), "cart", parent_->jmg_, parent_->ee_link_, rvt::BLUE));
  imarker_cartesian_->setIMarkerCallback(
      std::bind(&CartPathPlanner::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));

  // Set visual tools
  imarker_cartesian_->getVisualTools()->setMarkerTopic(nh_.getNamespace() + "/cartesian_trajectory");
  imarker_cartesian_->getVisualTools()->loadMarkerPub(true);
  imarker_cartesian_->getVisualTools()->deleteAllMarkers();
  imarker_cartesian_->getVisualTools()->setManualSceneUpdating(true);

  ROS_INFO_STREAM_NAMED(name_, "CartPathPlanner Ready.");
}

void CartPathPlanner::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                         const Eigen::Affine3d &feedback_pose)
{
  imarker_state_ = imarker_cartesian_->getRobotState();
  computePath();
}

bool CartPathPlanner::computePath()
{
  // Plan cartesian path
  trajectory_.clear();
  Eigen::Vector3d rotated_direction;
  rotated_direction << 0, -1, 0;
  double desired_distance = 0.5;  // Distance to move
  double max_step = 0.01;  // Resolution of trajectory, the maximum distance in Cartesian space between consecutive
                           // points on the resulting path

  // this is the Cartesian pose we start from, and we move in the direction indicated
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);
  trajectory_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*imarker_state_)));

  // The target pose is built by applying a translation to the start pose for the desired direction and distance
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

  imarker_cartesian_->getVisualTools()->enableBatchPublishing();
  imarker_cartesian_->getVisualTools()->deleteAllMarkers();
  for (unsigned int i = 1; i <= steps; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

    // Visualize
    imarker_cartesian_->getVisualTools()->publishArrow(pose, rvt::ORANGE, rvt::SMALL, 0.02, i);

    if (robot_state.setFromIK(parent_->jmg_, pose, parent_->ee_link_->getName(), 1, 0.0))
    {
      trajectory_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_state)));

      // compute the distance to the previous point (infinity norm)
      double dist_prev_point = trajectory_.back()->distance(*trajectory_[trajectory_.size() - 2], parent_->jmg_);
      dist_vector.push_back(dist_prev_point);
      total_dist += dist_prev_point;
    }
    else
      break;
    last_valid_percentage = percentage;
  }
  imarker_cartesian_->getVisualTools()->triggerBatchPublishAndDisable();

  // std::cout << "last_valid_percentage: " << last_valid_percentage << std::endl;

  if (!last_valid_percentage)
  {
    ROS_WARN_STREAM_NAMED(name_, "No solution found");
    return false;
  }

  // Visualize path
  // double speed = 0.001;
  // bool blocking = true;
  // imarker_cartesian_->getVisualTools()->publishTrajectoryPath(trajectory_, parent_->jmg_, speed, blocking);

  return true;
}

bool CartPathPlanner::getTrajectory(std::vector<moveit::core::RobotStatePtr> &trajectory)
{
  if (trajectory_.empty())
  {
    imarker_state_ = imarker_cartesian_->getRobotState();
    computePath();
  }

  if (trajectory_.empty())
    return false;

  trajectory = trajectory_;
  return true;
}

}  // namespace curie_demos
