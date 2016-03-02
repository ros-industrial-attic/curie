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
   Desc:   Demo dual arm manipulation
*/

#ifndef HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
#define HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_boilerplate/moveit_base.h>
#include <moveit/ompl/parameterization/model_based_state_space.h>
#include <moveit/ompl/model_based_planning_context.h>

// OMPL
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/bolt/Bolt.h>
#include <ompl_visual_tools/ompl_visual_tools.h>

// this package
#include <hilgendorf_moveit_demos/process_mem_usage.h>

namespace hilgendorf_moveit_demos
{
class HilgendorfDemos : public moveit_boilerplate::MoveItBase
{
public:
  /**
   * \brief Constructor
   */
  HilgendorfDemos() : MoveItBase(), nh_("~"), name_("hilgendorf_demos"), total_duration_(0.0), total_runs_(0), total_failures_(0)

  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "use_experience", use_experience_);
    error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
    error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
    error += !rosparam_shortcuts::get(name_, rpnh, "sparse_delta", sparse_delta_);
    error += !rosparam_shortcuts::get(name_, rpnh, "save_database", save_database_);
    error += !rosparam_shortcuts::get(name_, rpnh, "skip_solving", skip_solving_);
    // Visualize
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/playback_trajectory", visualize_playback_trajectory_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/grid_generation", visualize_grid_generation_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
    // Debug
    error += !rosparam_shortcuts::get(name_, rpnh, "debug/print_trajectory", debug_print_trajectory_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Initialize MoveIt base
    MoveItBase::init(nh_);

    // Load 2 more visual tools
    loadVisualTools();

    // Load 2 more robot states
    start_state_.reset(new moveit::core::RobotState(*current_state_));
    goal_state_.reset(new moveit::core::RobotState(*current_state_));
    from_state_.reset(new moveit::core::RobotState(*current_state_));
    to_state_.reset(new moveit::core::RobotState(*current_state_));

    // Get the two arms jmg
    // right_arm_ = robot_model_->getJointModelGroup("both_arms");
    right_arm_ = robot_model_->getJointModelGroup("right_arm");
    ee_link_ = robot_model_->getLinkModel("right_robotiq_85_right_finger_tip_link");

    double vm1, rss1;
    process_mem_usage(vm1, rss1);
    ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");

    // Load planning
    if (!loadPlanningContext())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
      return;
    }

    ROS_INFO_STREAM_NAMED(name_, "HilgendorfDemos Ready.");
  }

  bool loadPlanningContext()
  {
    // Note: we create a fake planning request in order to get the desired planning context

    // Create motion planning request
    planning_interface::MotionPlanRequest request;
    planning_interface::MotionPlanResponse result;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*current_state_, request.start_state);

    // Goal constraint
    double tolerance_pose = 0.001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*current_state_, right_arm_, tolerance_pose);
    request.goal_constraints.push_back(goal_constraint);

    // Other settings
    request.group_name = right_arm_->getName();
    request.use_experience = use_experience_;
    request.experience_method = experience_planner_;

    // Set planning space
    setWorkspace(request);

    // Load correct planning plugin
    if (!loadPlanningPlugin())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load planner manager, cannot plan.");
      return false;
    }

    // Create the planning scene - TODO use our global one?
    planning_scene::PlanningScenePtr temp_planning_scene;
    temp_planning_scene.reset(new planning_scene::PlanningScene(robot_model_));

    // Get planning context
    ROS_INFO_STREAM_NAMED(name_, "Getting planning context");
    planning_context_ = planner_manager_->getPlanningContext(temp_planning_scene, request); //, result.error_code_);

    // Error check
    if (!planning_context_)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Count not find planning_context_");
      return false;
    }

    // Get references to various parts of the Bolt framework
    mb_planning_context_ =
      boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_);
    mb_state_space_ = mb_planning_context_->getOMPLStateSpace();
    if (use_experience_)
    {
      experience_setup_ =
        boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mb_planning_context_->getOMPLSimpleSetup());
      bolt_setup_ = boost::dynamic_pointer_cast<ompl::tools::Bolt>(experience_setup_);
    }

    // Add custom distance function
    mb_state_space_->setDistanceFunction(boost::bind(&HilgendorfDemos::customDistanceFunction, this, _1, _2));

    // Set state space
    visual_ompl1_->setStateSpace(mb_state_space_);
    visual_ompl2_->setStateSpace(mb_state_space_);
    visual_ompl3_->setStateSpace(mb_state_space_);

    // Set visualization callbacks
    if (use_experience_)
    {
      bolt_setup_->getExperienceDB()->setViz2Callbacks(visual_ompl3_->getVizStateCallback(),
                                                       visual_ompl3_->getVizEdgeCallback(),
                                                       visual_ompl3_->getVizTriggerCallback());
      bolt_setup_->getRetrieveRepairPlanner().setVizCallbacks(visual_ompl3_->getVizStateCallback(),
                                                              visual_ompl3_->getVizEdgeCallback(),
                                                              visual_ompl3_->getVizTriggerCallback());

      // Set planner settings
      bolt_setup_->getExperienceDB()->sparseDelta_ = sparse_delta_;
      bolt_setup_->getExperienceDB()->visualizeGridGeneration_ = visualize_grid_generation_;
      bolt_setup_->getExperienceDB()->setSavingEnabled(save_database_);
    }

    double vm1, rss1;
    process_mem_usage(vm1, rss1);
    ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");

    // Load database or generate new grid
    if (use_experience_)
    {
      ROS_INFO_STREAM_NAMED(name_, "Loading or generating grid");
      bolt_setup_->loadOrGenerate();
      bolt_setup_->saveIfChanged();
    }

    double vm2, rss2;
    process_mem_usage(vm2, rss2);
    ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm2 << " MB | RSS: " << rss2 << " MB");
    ROS_INFO_STREAM_NAMED(name_, "Current memory diff        - VM: "
                          << vm2 - vm1 << " MB | RSS: " << rss2 - rss1 << " MB");

    // Show database
    //bolt_setup_->getExperienceDB()->displayDatabase();

    return true;
  }

  void testRandomStates()
  {
    ompl::base::State *random_state = mb_state_space_->allocState();

    std::size_t successful_connections = 0;
    for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
    {
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      ROS_INFO_STREAM_NAMED(name_, "Testing random state " << run_id);

      // Generate random state
      getRandomState(start_state_);

      // Visualize
      visual_start_->publishRobotState(start_state_, rvt::GREEN);

      // Convert to ompl
      mb_state_space_->copyToOMPLState(random_state, *start_state_);

      // Test
      const ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(60.0);
      bool result = bolt_setup_->getRetrieveRepairPlanner().canConnect(random_state, ptc);
      if (result)
        successful_connections++;

      ROS_ERROR_STREAM_NAMED(name_, "Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
    }

    mb_state_space_->freeState(random_state);
  }

  void runRandomProblems()
  {
    if (skip_solving_)
    {
      ROS_INFO_STREAM_NAMED(name_, "Skipping solving by request of rosparam");
      return;
    }

    // Start solving multiple times
    for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
    {
      ROS_INFO_STREAM_NAMED(name_, "Starting planning run " << run_id);

      // Generate start/goal pair
      getRandomState(start_state_);
      getRandomState(goal_state_);

      // Visualize
      if (visualize_start_goal_states_)
        visualizeStartGoal();

      // Plan from start to goal
      generateRequest(start_state_, goal_state_);

      // Check if time to end loop
      if (!ros::ok())
        return;
    }

    // Finishing up
    if (use_experience_)
    {
      ROS_INFO_STREAM_NAMED(name_,"Saving experience db...");
      experience_setup_->saveIfChanged();
    }

    // Stats
    ROS_ERROR_STREAM_NAMED(name_, "Average solving time: " << (total_duration_ / total_runs_));
  }

  bool generateRequest(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
  {
    // Create motion planning request
    planning_interface::MotionPlanRequest request;
    planning_interface::MotionPlanResponse result;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*start_state, request.start_state);

    // Goal constraint
    double tolerance_pose = 0.001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal_state, right_arm_, tolerance_pose);
    request.goal_constraints.push_back(goal_constraint);

    // Other request properties
    request.group_name = right_arm_->getName();
    request.num_planning_attempts = 1;   // this must be one else it threads and doesn't use lightning/thunder correctly
    request.allowed_planning_time = 20;  // second
    request.use_experience = use_experience_;
    request.experience_method = experience_planner_;

    bool verbose = false;
    if (verbose)
      std::cout << "Planning request:\n" << request << std::endl;

    // Set planning space
    setWorkspace(request);

    // Benchmark runtime
    ros::Time start_time = ros::Time::now();

    // SOLVE
    if (!planWithContext(planning_scene_, request, result))
      total_failures_++;

    // Benchmark runtime
    double duration = (ros::Time::now() - start_time).toSec();
    total_duration_ += duration;
    total_runs_++;
    ROS_WARN_STREAM_NAMED(name_, "planWithContext() total time: " << duration
                          << " sec, average: " << total_duration_ / total_runs_
                          << ", precent failure: " << double(total_failures_) / total_runs_ * 100.0 << "%");

    // Check if time to end
    if (!ros::ok())
      exit(0);

    // Check that the planning was successful
    if (result.error_code_.val != result.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return false;
    }
    ROS_INFO_STREAM_NAMED(name_, "Computed Plan Successfully! ---------------------------------");

    // Get the trajectory
    moveit_msgs::MotionPlanResponse response;
    response.trajectory = moveit_msgs::RobotTrajectory();
    result.getMessage(response);
    moveit_msgs::RobotTrajectory &traj = response.trajectory;

    // Parameterize trajectory
    for (std::size_t i = 1; i < traj.joint_trajectory.points.size(); ++i)
    {
      traj.joint_trajectory.points[i].time_from_start =
        traj.joint_trajectory.points[i-1].time_from_start + ros::Duration(0.25);
    }

    // Clear Rviz
    visual_ompl3_->hideRobot();
    visual_ompl3_->deleteAllMarkers();

    // Debug Output trajectory
    if (debug_print_trajectory_)
      std::cout << "Trajectory debug:\n " << traj << std::endl;

    // Visualize the trajectory
    if (visualize_playback_trajectory_)
    {
      ROS_INFO("Visualizing the trajectory");
      const bool wait_for_trajetory = true;
      visual_tools_->publishTrajectoryPath(traj, current_state_, wait_for_trajetory);
      ros::Duration(2).sleep();
    }

    // Process the popularity
    if (use_experience_)
    {
      experience_setup_->doPostProcessing();
    }

    // Visualize the doneness
    std::cout << std::endl;

    return true;
  }

  bool planWithContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                           const planning_interface::MotionPlanRequest& request,
                           planning_interface::MotionPlanResponse& result)
  {
    // Update the planning context (should return same one)
    ROS_INFO_STREAM_NAMED(name_, "Getting planning context");
    planning_context_ = planner_manager_->getPlanningContext(planning_scene, request, result.error_code_);

    // Error check
    if (!planning_context_)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Count not find planning_context_");
      return false;
    }

    ROS_INFO_STREAM_NAMED(name_, "Starting to solve");
    if (!planning_context_->solve(result))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to solve problem");

      // Show the unsolvable problem, if its not already showing
      if (!visualize_start_goal_states_)
        visualizeStartGoal();

      return false;
    }

    // Verify path is good
    return true;  // checkPathSolution(planning_scene, request, result);
  }

  bool loadPlanningPlugin()
  {
    if (planner_manager_)
    {
      ROS_DEBUG_STREAM_NAMED("loadPlanningPlugin", "Already loaded planner");
      return true;
    }

    // load the planning plugin
    std::string planner_plugin_name_ = "";
    try
    {
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
      return false;
    }

    std::vector<std::string> classes;
    if (planner_plugin_loader_)
      classes = planner_plugin_loader_->getDeclaredClasses();
    if (planner_plugin_name_.empty() && classes.size() == 1)
    {
      planner_plugin_name_ = classes[0];
      ROS_INFO("No '~planning_plugin' parameter specified, but only '%s' planning plugin is available. Using that one.",
               planner_plugin_name_.c_str());
    }
    if (planner_plugin_name_.empty() && classes.size() > 1)
    {
      planner_plugin_name_ = classes[0];
      ROS_INFO("Multiple planning plugins available. You should specify the '~planning_plugin' parameter. Using '%s' "
               "for now.",
               planner_plugin_name_.c_str());
    }
    try
    {
      planner_manager_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));

      // Initialize the planner
      if (!planner_manager_->initialize(robot_model_, nh_.getNamespace()))
        throw std::runtime_error("Unable to initialize planning plugin");
      ROS_INFO_STREAM("Using planning interface '" << planner_manager_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner '"
                       << planner_plugin_name_ << "': " << ex.what() << std::endl
                       << "Available plugins: " << boost::algorithm::join(classes, ", "));
      return false;
    }

    // Error check
    if (!planner_manager_)
    {
      ROS_ERROR("No planning plugin loaded. Cannot plan.");
      return false;
    }

    return true;
  }

  bool checkPathSolution(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const planning_interface::MotionPlanRequest& request,
                         planning_interface::MotionPlanResponse& result)
  {
    // Check solution
    if (!result.trajectory_)
    {
      ROS_ERROR_STREAM_NAMED(name_, "No trajectory");
      return false;
    }

    // Debug
    const std::size_t state_count = result.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with " << state_count << " states");

    std::vector<std::size_t> index;
    if (!planning_scene->isPathValid(*result.trajectory_, request.path_constraints, request.group_name, false, &index))
    {
      if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
        ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
      else
      {
        result.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;

        // display error messages
        std::stringstream ss;
        for (std::size_t i = 0; i < index.size(); ++i)
          ss << index[i] << " ";
        ROS_ERROR_STREAM("Computed path is not valid. Invalid states at index locations: [ "
                         << ss.str() << "] out of " << state_count << ". Explanations follow in command line.");

        // call validity checks in verbose mode for the problematic states
        visualization_msgs::MarkerArray arr;
        for (std::size_t i = 0; i < index.size(); ++i)
        {
          // check validity with verbose on
          const robot_state::RobotState& robot_state = result.trajectory_->getWayPoint(index[i]);
          planning_scene->isStateValid(robot_state, request.path_constraints, request.group_name, true);

          // compute the contacts if any
          collision_detection::CollisionRequest c_req;
          collision_detection::CollisionResult c_res;
          c_req.contacts = true;
          c_req.max_contacts = 10;
          c_req.max_contacts_per_pair = 3;
          c_req.verbose = false;
          planning_scene->checkCollision(c_req, c_res, robot_state);
          /*
            if (c_res.contact_count > 0)
            {
            visualization_msgs::MarkerArray arr_i;
            collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(),
            c_res.contacts);
            arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
            }
          */
        }
        ROS_ERROR_STREAM("Completed listing of explanations for invalid states.");
        // if (!arr.markers.empty())
        //  contacts_publisher_.publish(arr);
      }
    }
    return true;
  }

  void setWorkspace(planning_interface::MotionPlanRequest& request)
  {
    // Parameters for the workspace that the planner should work inside relative to center of robot
    double workspace_size = 2;
    request.workspace_parameters.header.frame_id = robot_model_->getModelFrame();
    request.workspace_parameters.min_corner.x =
        current_state_->getVariablePosition("virtual_joint/trans_x") - workspace_size;
    request.workspace_parameters.min_corner.y =
        current_state_->getVariablePosition("virtual_joint/trans_y") - workspace_size;
    request.workspace_parameters.min_corner.z = 0;  // floor
    request.workspace_parameters.max_corner.x =
        current_state_->getVariablePosition("virtual_joint/trans_x") + workspace_size;
    request.workspace_parameters.max_corner.y =
        current_state_->getVariablePosition("virtual_joint/trans_y") + workspace_size;
    request.workspace_parameters.max_corner.z =
        current_state_->getVariablePosition("virtual_joint/trans_z") + workspace_size;
    //visual_tools_->publishWorkspaceParameters(request.workspace_parameters);
  }

  bool getRandomState(moveit::core::RobotStatePtr& robot_state)
  {
    static const std::size_t MAX_ATTEMPTS = 100;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
      robot_state->setToRandomPositions(right_arm_);
      robot_state->update();

      // Error check
      bool check_verbose = false;
      if (planning_scene_->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                           // collision check, "" is everything
      {
        //ROS_DEBUG_STREAM_NAMED(name_, "Found valid random robot state after " << i << " attempts");
        return true;
      }
    }

    ROS_ERROR_STREAM_NAMED(name_, "Unable to find valid random robot state");
    exit(-1);
    return false;
  }

  void loadVisualTools()
  {
    // Note: this is in addition to the moveit_boilerplate visual_tools

    std::string namesp = nh_.getNamespace();
    visual_ompl1_.reset(
        new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/start_markers", robot_model_));
    //visual_ompl1_->deleteAllMarkers();
    visual_ompl1_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_ompl1_->loadRobotStatePub(namesp + "/start_state");
    visual_ompl1_->setManualSceneUpdating(true);
    visual_ompl1_->hideRobot();  // show that things have been reset
    visual_start_ = visual_ompl1_;

    visual_ompl2_.reset(
        new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/goal_markers", robot_model_));
    //visual_ompl2_->deleteAllMarkers();
    visual_ompl2_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_ompl2_->loadRobotStatePub(namesp + "/goal_state");
    visual_ompl2_->setManualSceneUpdating(true);
    visual_ompl2_->hideRobot();  // show that things have been reset
    visual_goal_ = visual_ompl2_; // copy for use by Moveit

    visual_ompl3_.reset(
        new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/ompl_markers", robot_model_));
    visual_ompl3_->deleteAllMarkers();
    visual_ompl3_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_ompl3_->loadRobotStatePub(namesp + "/ompl_state");
    visual_ompl3_->setManualSceneUpdating(true);
    visual_ompl3_->hideRobot();  // show that things have been reset
  }

  void visualizeStartGoal()
  {
    visual_start_->publishRobotState(start_state_, rvt::GREEN);
    visual_goal_->publishRobotState(goal_state_, rvt::ORANGE);

    // Show values and limits
    std::cout << "Start: " << std::endl;
    visual_start_->showJointLimits(start_state_);
    std::cout << "Goal: " << std::endl;
    visual_start_->showJointLimits(goal_state_);
  }

  double customDistanceFunction(const ompl::base::State *state1, const ompl::base::State *state2)
  {
    std::cout << "no one should call this " << std::endl;
    exit(-1);

    mb_state_space_->copyToRobotState(*from_state_, state1);
    mb_state_space_->copyToRobotState(*to_state_, state2);

    const Eigen::Affine3d from_pose = from_state_->getGlobalLinkTransform(ee_link_);
    const Eigen::Affine3d to_pose = to_state_->getGlobalLinkTransform(ee_link_);

    return getPoseDistance(from_pose, to_pose);
  }

  double getPoseDistance(const Eigen::Affine3d &from_pose, const Eigen::Affine3d &to_pose)
  {
    const double translation_dist = (from_pose.translation() - to_pose.translation()).norm();
    //const double distance_wrist_to_finger = 0.25; // meter

    const Eigen::Quaterniond from(from_pose.rotation());
    const Eigen::Quaterniond to(to_pose.rotation());

    //std::cout << "From: " << from.x() << ", " << from.y() << ", " << from.z() << ", " << from.w() << std::endl;
    //std::cout << "To: " << to.x() << ", " << to.y() << ", " << to.z() << ", " << to.w() << std::endl;

    double rotational_dist = arcLength(from, to); // * distance_wrist_to_finger;

    std::cout << "  Translation_Dist: " << std::fixed << std::setprecision(4) << translation_dist
              << " rotational_dist: " << rotational_dist << std::endl;

    return rotational_dist + translation_dist;
  }

  double arcLength(const Eigen::Quaterniond &from, const Eigen::Quaterniond &to)
  {
    static const double MAX_QUATERNION_NORM_ERROR = 1e-9;
    double dq = fabs(from.x() * to.x() + from.y() * to.y() + from.z() * to.z() + from.w() * to.w());
    if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
      return 0.0;
    else
      return acos(dq);
  }

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl1_;
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl2_;
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl3_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_start_; // Clone of ompl1
  moveit_visual_tools::MoveItVisualToolsPtr visual_goal_; // Clone of ompl2

  // Robot states
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr goal_state_;
  moveit::core::RobotStatePtr from_state_;
  moveit::core::RobotStatePtr to_state_;

  // Planning groups
  moveit::core::JointModelGroup* right_arm_;
  moveit::core::LinkModel *ee_link_;

  // Planning Plugin Components
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_manager_;

  // Operation settings
  bool use_experience_;
  std::string experience_planner_;
  std::size_t planning_runs_;
  double sparse_delta_;
  bool save_database_;
  bool skip_solving_;

  // Debug and display preferences
  bool visualize_playback_trajectory_;
  bool visualize_grid_generation_;
  bool visualize_start_goal_states_;
  bool debug_print_trajectory_;

  // Remember the planning context even after solving is done
  ompl::tools::ExperienceSetupPtr experience_setup_;
  planning_interface::PlanningContextPtr planning_context_;
  moveit_ompl::ModelBasedPlanningContextPtr mb_planning_context_;
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space_;
  ompl::tools::BoltPtr bolt_setup_;

  // Average planning time
  double total_duration_;
  std::size_t total_runs_;
  std::size_t total_failures_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<HilgendorfDemos> HilgendorfDemosPtr;
typedef boost::shared_ptr<const HilgendorfDemos> HilgendorfDemosConstPtr;

}  // namespace hilgendorf_moveit_demos

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "hilgendorf_demos");
  ROS_INFO_STREAM_NAMED("main", "Starting HilgendorfDemos...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  hilgendorf_moveit_demos::HilgendorfDemos server;
  //server.runRandomProblems();
  //server.testRandomStates();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif  // HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
