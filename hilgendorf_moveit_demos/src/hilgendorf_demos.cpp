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
  HilgendorfDemos() : MoveItBase(), nh_("~"), name_("hilgendorf_demos")
  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "use_experience", use_experience_);
    error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
    error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
    //error += !rosparam_shortcuts::get(name_, rpnh, "expand_neighborhood_rate", expand_neighborhood_rate_);
    // Visualize
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/playback_trajectory", viz_playback_trajectory_);
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

    // Get the two arms jmg
    // both_arms_ = robot_model_->getJointModelGroup("both_arms");
    both_arms_ = robot_model_->getJointModelGroup("right_arm");

    // showJointLimits(both_arms_);

    ROS_INFO_STREAM_NAMED(name_, "HilgendorfDemos Ready.");
  }

  void run()
  {
    // Load planning

    for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
    {
      ROS_INFO_STREAM_NAMED(name_, "Starting planning run " << run_id);

      // Generate start/goal pair
      getRandomState(start_state_);
      getRandomState(goal_state_);

      // Visualize
      visual_start_->publishRobotState(start_state_, rvt::GREEN);
      visual_goal_->publishRobotState(goal_state_, rvt::ORANGE);

      // Plan from start to goal
      if (!generatePlan(start_state_, goal_state_))
      {
        return;
      }

      // Error check
      if (!planning_context_)
      {
        ROS_ERROR_STREAM_NAMED(name_, "No planning context handle point available");
      }

      // Wait for next run
      ros::Duration(2.0).sleep();
    }
  }

  bool generatePlan(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
  {
    // Create motion planning request
    planning_interface::MotionPlanRequest request;
    planning_interface::MotionPlanResponse result;

    // Start state
    moveit::core::robotStateToRobotStateMsg(*start_state, request.start_state);

    // Goal constraint
    double tolerance_pose = 0.001;
    moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(*goal_state, both_arms_, tolerance_pose);
    std::cout << "goal_constraint: \n" << goal_constraint << std::endl;
    request.goal_constraints.push_back(goal_constraint);

    // Other settings
    request.planner_id = "RRTConnectkConfigDefault";
    //ROS_INFO_STREAM_NAMED(name_, "Planning with planner " << request.planner_id);
    request.group_name = both_arms_->getName();
    request.num_planning_attempts = 1;   // this must be one else it threads and doesn't use lightning/thunder correctly
    request.allowed_planning_time = 20;  // second
    request.use_experience = use_experience_;
    request.experience_method = experience_planner_;

    std::cout << "request:\n" << request << std::endl;
    // Set planning space
    setWorkspace(request);

    // Call pipeline
    std::vector<std::size_t> dummy_pipeline;

    // SOLVE
    {
      // Create the planning scene
      planning_scene::PlanningScenePtr planning_scene2;
      planning_scene2.reset(new planning_scene::PlanningScene(robot_model_));

      // Lock the planning scene
      // boost::scoped_ptr<psm::LockedPlanningSceneRO> ls;
      // ls.reset(new psm::LockedPlanningSceneRO(planning_scene_monitor_));
      planWithoutPipeline(planning_scene2, request, result, dummy_pipeline);
    }

    // Check that the planning was successful
    if (result.error_code_.val != result.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully =============================================");
      return false;
    }

    std::cout << std::endl;
    ROS_INFO_STREAM_NAMED(name_, "Computed Plan Successfully! ---------------------------------");
    std::cout << std::endl;

    // Get the trajectory
    moveit_msgs::MotionPlanResponse response;
    response.trajectory = moveit_msgs::RobotTrajectory();
    result.getMessage(response);

    // Debug Output trajectory
    if (debug_print_trajectory_)
    {
      std::cout << "Trajectory debug:\n " << response.trajectory << std::endl;
    }

    // Visualize the trajectory
    if (viz_playback_trajectory_)
    {
      ROS_INFO("Visualizing the trajectory");
      bool wait_for_trajetory = true;
      visual_tools_->publishTrajectoryPath(response.trajectory, current_state_, wait_for_trajetory);
    }

    return true;
  }

  bool planWithoutPipeline(const planning_scene::PlanningSceneConstPtr& planning_scene,
                           const planning_interface::MotionPlanRequest& request,
                           planning_interface::MotionPlanResponse& result,
                           std::vector<std::size_t>& adapter_added_state_index)
  {
    // Load correct plugin
    if (!loadPlanningPlugin())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load planner manager, cannot plan.");
      return false;
    }

    // Get planning context
    ROS_INFO_STREAM_NAMED(name_, "Getting planning context");
    planning_context_ = planner_manager_->getPlanningContext(planning_scene, request, result.error_code_);

    // Error check
    if (!planning_context_)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Count not find planning_context_");
      return false;
    }

    // Get references to various parts of the Bolt framework
    mb_planning_context_ =
      boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_);
    experience_setup_ =
      boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mb_planning_context_->getOMPLSimpleSetup());
    mb_state_space_ = mb_planning_context_->getOMPLStateSpace();
    ompl::tools::Bolt &bolt_setup = static_cast<ompl::tools::Bolt &>(*experience_setup_);
    ompl::geometric::BoltRetrieveRepair &bolt_retireve_repair = bolt_setup.getRetrieveRepairPlanner();

    // Set state space
    visual_ompl1_->setStateSpace(mb_state_space_);

    // Set visualization callbacks
    bolt_setup.getExperienceDB()->setViz2Callbacks(visual_ompl1_->getVizStateCallback(),
                                                    visual_ompl1_->getVizEdgeCallback(),
                                                    visual_ompl1_->getVizTriggerCallback());

    // Set planner settings
    // TODO

    // Load database or generate new grid
    ROS_INFO_STREAM_NAMED(name_, "Loading or generating grid");
    bolt_setup.loadOrGenerate();

    double vm, rss;
    process_mem_usage(vm, rss);
    std::cout << "VM: " << vm << " MB    |    RSS: " << rss << " MB" << std::endl;

    // Solve
    ROS_INFO_STREAM_NAMED(name_, "Starting to solve");
    if (!planning_context_->solve(result))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to solve problem");
      return false;
    }

    // Process the popularity
    experience_setup_->doPostProcessing();

    // Setup
    ROS_INFO_STREAM_NAMED(name_,"Saving experience db...");
    experience_setup_->saveIfChanged();

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
    visual_tools_->publishWorkspaceParameters(request.workspace_parameters);
  }

  bool getRandomState(moveit::core::RobotStatePtr& robot_state)
  {
    static const std::size_t MAX_ATTEMPTS = 100;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
      robot_state->setToRandomPositions(both_arms_);
      robot_state->update();

      // Error check
      bool check_verbose = false;
      if (planning_scene_->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                           // collision check, "" is everything
      {
        ROS_DEBUG_STREAM_NAMED(name_, "Found valid random robot state after " << i << " attempts");
        return true;
      }
    }

    ROS_ERROR_STREAM_NAMED(name_, "Unable to find valid random robot state");
    return false;
  }

  void loadVisualTools()
  {
    // Note: this is in addition to the moveit_boilerplate visual_tools

    std::string namesp = nh_.getNamespace();
    visual_ompl1_.reset(
        new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/start_markers", robot_model_));
    visual_ompl1_->deleteAllMarkers();
    visual_ompl1_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_ompl1_->loadRobotStatePub(namesp + "/start_state");
    visual_ompl1_->setAlpha(0.8);
    visual_ompl1_->setManualSceneUpdating(true);
    visual_ompl1_->hideRobot();  // show that things have been reset
    visual_start_ = visual_ompl1_;

    visual_ompl2_.reset(
        new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/goal_markers", robot_model_));
    visual_ompl2_->deleteAllMarkers();
    visual_ompl2_->setPlanningSceneMonitor(planning_scene_monitor_);
    visual_ompl2_->loadRobotStatePub(namesp + "/goal_state");
    visual_ompl2_->setAlpha(0.8);
    visual_ompl2_->setManualSceneUpdating(true);
    visual_ompl2_->hideRobot();  // show that things have been reset
    visual_goal_ = visual_ompl2_; // copy for use by Moveit
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
  moveit_visual_tools::MoveItVisualToolsPtr visual_start_; // Clone of ompl1
  moveit_visual_tools::MoveItVisualToolsPtr visual_goal_; // Clone of ompl2

  // Robot states
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr goal_state_;

  // Planning groups
  moveit::core::JointModelGroup* both_arms_;

  // Planning Plugin Components
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_manager_;

  // Operation settings
  bool use_experience_;
  std::string experience_planner_;
  std::size_t planning_runs_;
  //double expand_neighborhood_rate_;

  // Debug and Display preferences
  bool viz_playback_trajectory_;
  bool debug_print_trajectory_;

  // Remember the planning context even after solving is done
  ompl::tools::ExperienceSetupPtr experience_setup_;
  planning_interface::PlanningContextPtr planning_context_;
  moveit_ompl::ModelBasedPlanningContextPtr mb_planning_context_;
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space_;

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
  server.run();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif  // HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
