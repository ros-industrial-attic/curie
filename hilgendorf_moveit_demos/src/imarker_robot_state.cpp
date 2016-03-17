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
   Desc:   Class to encapsule a visualized robot state that can be controlled using an interactive marker
*/

// Conversions
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

// this package
#include <hilgendorf_moveit_demos/imarker_robot_state.h>

namespace hilgendorf_moveit_demos
{

IMarkerRobotState::IMarkerRobotState(psm::PlanningSceneMonitorPtr planning_scene_monitor, const std::string &imarker_name,
                    const moveit::core::JointModelGroup *jmg, rvt::colors color)
    : name_(imarker_name), nh_("~"), planning_scene_monitor_(planning_scene_monitor), jmg_(jmg), color_(color)
  {
    imarker_topic_ = nh_.getNamespace() + "/" + imarker_name + "_imarker";

    // Load rosparams
    // ros::NodeHandle rpnh(nh_, name_);
    // std::size_t error = 0;
    // error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    // rosparam_shortcuts::shutdownIfError(name_, error);

    // Load Visual tools
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(
        planning_scene_monitor_->getRobotModel()->getModelFrame(), nh_.getNamespace() + "/" + imarker_name,
        planning_scene_monitor_->getRobotModel()));
    visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/imarker_" + imarker_name + "_state");

    // Load robot state
    imarker_state_.reset(new moveit::core::RobotState(planning_scene_monitor_->getRobotModel()));
    imarker_state_->setToDefaultValues();

    // Load previous pose from file
    Eigen::Affine3d start_pose = Eigen::Affine3d::Identity();
    getFilePath(file_path_, "imarker_" + name_ + ".csv", "ompl_storage");

    loadFromFile(start_pose, file_path_);

    // Create imarker
    initializeInteractiveMarkers(start_pose);

    // Show initial robot state
    solveIK(start_pose);

    ROS_INFO_STREAM_NAMED(name_, "IMarkerRobotState " << name_ << " Ready.");
  }

  void IMarkerRobotState::setIMarkerCallback(IMarkerCallback callback)
  {
    imarker_callback_ = callback;
  }

  void IMarkerRobotState::getPose(Eigen::Affine3d &pose)
  {
    pose = imarker_pose_;
  }

  moveit::core::RobotStatePtr IMarkerRobotState::getRobotState()
  {
    return imarker_state_;
  }

  bool IMarkerRobotState::loadFromFile(Eigen::Affine3d &pose, const std::string &file_name)
  {
    std::ifstream input_file(file_name);

    std::string line;
    std::string cell;
    std::vector<double> values;

    // For each line/row
    while (std::getline(input_file, line))
    {
      std::stringstream lineStream(line);

      // For each item/column
      for (std::size_t i = 0; i < 16; ++i)
      {
        // Value
        if (!std::getline(lineStream, cell, ','))
          ROS_ERROR_STREAM_NAMED(name_, "No value found for " << i);

        pose.data()[i] = atof(cell.c_str());
      }  // for

      break;  // TODO(davetcoleman): can currently only handle one pose at a time
    }         // while

    return true;
  }

  bool IMarkerRobotState::saveToFile(Eigen::Affine3d &pose, const std::string &file_name)
  {
    std::ofstream output_file(file_name);

    // For each item/column
    for (std::size_t i = 0; i < 16; ++i)
    {
      output_file << pose.data()[i] << ", ";
    }  // for

    output_file << std::endl;

    return true;
  }

  void IMarkerRobotState::iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    // Ignore if not pose update
    if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
      return;

    // Only allow one feedback to be processed at a time
    {
      boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
      if (!imarker_ready_to_process_)
      {
        return;
      }
      imarker_ready_to_process_ = false;
    }

    // Convert
    Eigen::Affine3d robot_ee_pose;
    tf::poseMsgToEigen(feedback->pose, robot_ee_pose);

    // Save pose to file if its been long enough
    if (time_since_last_save_ < ros::Time::now() - ros::Duration(1.0))
    {
      saveToFile(robot_ee_pose, file_path_);
      time_since_last_save_ = ros::Time::now();
    }

    // Offset ee pose forward, because interactive marker is a special thing in front of hand
    robot_ee_pose = robot_ee_pose * imarker_offset_;

    // Update robot
    solveIK(robot_ee_pose);

    // Redirect to base class
    if (imarker_callback_)
      imarker_callback_(feedback, robot_ee_pose);

    // Allow next feedback to be processed
    {
      boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
      imarker_ready_to_process_ = true;
    }
  }

  void IMarkerRobotState::solveIK(Eigen::Affine3d &pose)
  {
    // Attempt to set robot to new pose
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, name_, "Setting from IK");
    if (imarker_state_->setFromIK(jmg_, pose))
    {
      ROS_INFO_STREAM_NAMED(name_, "Solved IK");
      visual_tools_->publishRobotState(imarker_state_, color_);
    }
    else
      ROS_DEBUG_STREAM_NAMED(name_, "Failed to set IK");
  }

  void IMarkerRobotState::initializeInteractiveMarkers(const Eigen::Affine3d &pose)
  {
    // Move marker to tip of fingers
    imarker_pose_ = pose * imarker_offset_.inverse();

    // Convert
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(imarker_pose_, pose_msg);

    // Server
    imarker_server_.reset(new interactive_markers::InteractiveMarkerServer(imarker_topic_, "", false));

    // Menu
    // menu_handler_.insert("Reset", boost::bind(&iMarkerCallback, this, _1));
    // interactive_markers::MenuHandler::EntryHandle sub_menu_handle1 = menu_handler_.insert("Recording");
    // menu_handler_.insert(sub_menu_handle1, "Add Current Pose", boost::bind(&iMarkerCallback, this, _1));
    // menu_handler_.insert(sub_menu_handle1, "Record IMarker Movements",
    //                      boost::bind(&iMarkerCallback, this, _1));
    // menu_handler_.insert(sub_menu_handle1, "Stop Recording IMarker",
    //                      boost::bind(&iMarkerCallback, this, _1));
    // interactive_markers::MenuHandler::EntryHandle sub_menu_handle2 = menu_handler_.insert("Playback");
    // menu_handler_.insert(sub_menu_handle2, "Play Trajectory", boost::bind(&iMarkerCallback, this, _1));
    // menu_handler_.insert(sub_menu_handle2, "Stop Trajectory", boost::bind(&iMarkerCallback, this, _1));
    // menu_handler_.insert(sub_menu_handle2, "Clear Trajectory", boost::bind(&iMarkerCallback, this, _1));

    // marker
    make6DofMarker(pose_msg);
    imarker_server_->applyChanges();
  }

  void IMarkerRobotState::updateIMarkerPose(const Eigen::Affine3d &pose)
  {
    // Move marker to tip of fingers
    imarker_pose_ = pose * imarker_offset_.inverse();
    sendUpdatedIMarkerPose();
  }

  void IMarkerRobotState::sendUpdatedIMarkerPose()
  {
    // Convert
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(imarker_pose_, pose_msg);

    imarker_server_->setPose(int_marker_.name, pose_msg);
    imarker_server_->applyChanges();
  }

  void IMarkerRobotState::make6DofMarker(const geometry_msgs::Pose &pose)
  {
    int_marker_.header.frame_id = "world";
    int_marker_.pose = pose;
    int_marker_.scale = 0.2;

    int_marker_.name = "6dof_teleoperation";
    int_marker_.description = name_;

    // insert a box
    makeBoxControl(int_marker_);

    // insert mesh of robot's end effector
    // makeEEControl(int_marker_);
    // int_marker_.controls[0].interaction_mode = InteractiveMarkerControl::MENU;

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);

    imarker_server_->insert(int_marker_);
    imarker_server_->setCallback(int_marker_.name, boost::bind(&IMarkerRobotState::iMarkerCallback, this, _1));
    // menu_handler_.apply(*imarker_server_, int_marker_.name);
  }

  visualization_msgs::InteractiveMarkerControl& IMarkerRobotState::makeBoxControl(visualization_msgs::InteractiveMarker &msg)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.3;  // x direction
    marker.scale.y = msg.scale * 0.10;  // y direction
    marker.scale.z = msg.scale * 0.10;   // height
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    msg.controls.push_back(control);

    return msg.controls.back();
  }

bool IMarkerRobotState::getFilePath(std::string &file_path, const std::string &file_name,
                 const std::string &subdirectory) const

{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path rootPath;
  if (!std::string(getenv("HOME")).empty())
    rootPath = fs::path(getenv("HOME"));  // Support Linux/Mac
  else if (!std::string(getenv("HOMEPATH")).empty())
    rootPath = fs::path(getenv("HOMEPATH"));  // Support Windows
  else
  {
    ROS_WARN("Unable to find a home path for this computer");
    rootPath = fs::path("");
  }

  rootPath = rootPath / fs::path(subdirectory);

  boost::system::error_code returnedError;
  fs::create_directories(rootPath, returnedError);

  if (returnedError)
  {
    // did not successfully create directories
    ROS_ERROR("Unable to create directory %s", subdirectory.c_str());
    return false;
  }

  // directories successfully created, append the group name as the file name
  rootPath = rootPath / fs::path(file_name);
  file_path = rootPath.string();
  //ROS_INFO_STREAM_NAMED(name_, "Setting database to " << file_path);

  return true;
}

  moveit_visual_tools::MoveItVisualToolsPtr IMarkerRobotState::getVisualTools()
  {
    return visual_tools_;
  }

}  // namespace hilgendorf_moveit_demos
