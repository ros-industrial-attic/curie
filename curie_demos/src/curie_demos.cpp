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

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// this package
#include <curie_demos/curie_demos.h>

namespace curie_demos
{
CurieDemos::CurieDemos()
  : MoveItBase(), nh_("~"), name_("curie_demos"), total_duration_(0.0), total_runs_(0), total_failures_(0)

{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "auto_run", auto_run_);
  error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
  error += !rosparam_shortcuts::get(name_, rpnh, "sparse_delta", sparse_delta_);
  error += !rosparam_shortcuts::get(name_, rpnh, "save_database", save_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "skip_solving", skip_solving_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  // Visualize
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/display_database", visualize_display_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/interpolated_trajectory", visualize_interpolated_trajectory_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/raw_trajectory", visualize_raw_trajectory_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/grid_generation", visualize_grid_generation_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/astar", visualize_astar_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/time_between_plans", visualize_time_between_plans_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/cart_neighbors", visualize_cart_neighbors_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/cart_path", visualize_cart_path_);
  // Debug
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/print_trajectory", debug_print_trajectory_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Initialize MoveIt base
  MoveItBase::init(nh_);

  // Load more visual tool objects
  loadVisualTools();

  // Load 2 more robot states
  moveit_start_.reset(new moveit::core::RobotState(*current_state_));
  moveit_goal_.reset(new moveit::core::RobotState(*current_state_));

  // Get the two arms jmg
  jmg_ = robot_model_->getJointModelGroup(planning_group_name_);
  ee_link_ = robot_model_->getLinkModel("right_gripper_target");

  double vm1, rss1;
  process_mem_usage(vm1, rss1);
  ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");

  // Create cartesian planner
  cart_path_planner_.reset(new CartPathPlanner(this));

  // Create start/goal state imarker
  imarker_start_.reset(new IMarkerRobotState(planning_scene_monitor_, "start", jmg_, ee_link_, rvt::GREEN));
  imarker_goal_.reset(new IMarkerRobotState(planning_scene_monitor_, "goal", jmg_, ee_link_, rvt::ORANGE));

  ros::Duration(1.0).sleep();

  // Wait until user does something
  if (!auto_run_)
    ros::spin();

  // TODO(davetcoleman): temp
  ros::spin();
  exit(0);

  // Load planning
  if (!loadOMPL())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
    return;
  }

  ROS_INFO_STREAM_NAMED(name_, "CurieDemos Ready.");
}

CurieDemos::~CurieDemos()
{
  // Free start and goal states
  space_->freeState(ompl_start_);
  space_->freeState(ompl_goal_);
}

bool CurieDemos::loadOMPL()
{
  moveit_ompl::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, jmg_);

  // Construct the state space we are planning in
  space_.reset(new moveit_ompl::ModelBasedStateSpace(mbss_spec));
  experience_setup_.reset(new ompl::tools::Bolt(space_));
  std::cout << "getSpaceInformation " << std::endl;
  si_ = experience_setup_->getSpaceInformation();

  // Add custom distance function
  // space_->setDistanceFunction(boost::bind(&CurieDemos::customDistanceFunction, this, _1, _2));

  // Set state validity checking for this space
  experience_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(
      new moveit_ompl::StateValidityChecker(planning_group_name_, si_, *current_state_, planning_scene_, space_)));

  // The interval in which obstacles are checked for between states
  // seems that it default to 0.01 but doesn't do a good job at that level
  si_->setStateValidityCheckingResolution(0.005);

  // Setup base OMPL stuff
  ROS_INFO_STREAM_NAMED(name_, "Setting up OMPL experience setup");
  experience_setup_->setup();
  assert(si_->isSetup());

  // Create start and goal states
  ompl_start_ = space_->allocState();
  ompl_goal_ = space_->allocState();

  // Set state space
  visual_ompl1_->setStateSpace(space_);
  visual_ompl2_->setStateSpace(space_);
  visual_ompl3_->setStateSpace(space_);

  // Set visualization callbacks
  experience_setup_->getExperienceDB()->setVizCallbacks(visual_ompl1_->getVizStateCallback(),
                                                        visual_ompl1_->getVizEdgeCallback(),
                                                        visual_ompl1_->getVizTriggerCallback());
  experience_setup_->getExperienceDB()->setViz2Callbacks(visual_ompl2_->getVizStateCallback(),
                                                         visual_ompl2_->getVizEdgeCallback(),
                                                         visual_ompl2_->getVizTriggerCallback());
  experience_setup_->getRetrieveRepairPlanner().setVizCallbacks(visual_ompl3_->getVizStateCallback(),
                                                                visual_ompl3_->getVizEdgeCallback(),
                                                                visual_ompl3_->getVizTriggerCallback());

  // Set planner settings
  experience_setup_->getExperienceDB()->visualizeAstar_ = visualize_astar_;
  experience_setup_->getExperienceDB()->sparseDelta_ = sparse_delta_;
  experience_setup_->getExperienceDB()->visualizeGridGeneration_ = visualize_grid_generation_;
  experience_setup_->getExperienceDB()->setSavingEnabled(save_database_);
  experience_setup_->getExperienceDB()->visualizeCartNeighbors_ = visualize_cart_neighbors_;
  experience_setup_->getExperienceDB()->visualizeCartPath_ = visualize_cart_path_;

  // Auto setup parameters (optional actually)
  // experience_setup_->enablePlanningFromRecall(use_recall_);
  // experience_setup_->enablePlanningFromScratch(use_scratch_);

  // Calibrate the color scale for visualization
  const bool invert_colors = true;
  visual_ompl1_->setMinMaxEdgeCost(0, 110, invert_colors);
  visual_ompl2_->setMinMaxEdgeCost(0, 110, invert_colors);
  visual_ompl3_->setMinMaxEdgeCost(0, 110, invert_colors);

  visual_ompl1_->setMinMaxEdgeRadius(0.001, 0.005);
  visual_ompl2_->setMinMaxEdgeRadius(0.001, 0.005);
  visual_ompl3_->setMinMaxEdgeRadius(0.001, 0.005);

  std::string file_path;
  getFilePath(file_path, planning_group_name_, "ros/ompl_storage");
  experience_setup_->setFilePath(file_path);  // this is here because its how we do it in moveit_ompl

  double vm1, rss1;
  process_mem_usage(vm1, rss1);
  ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");

  // Load database or generate new grid
  ROS_INFO_STREAM_NAMED(name_, "Loading or generating grid");
  experience_setup_->loadOrGenerate();
  experience_setup_->saveIfChanged();

  double vm2, rss2;
  process_mem_usage(vm2, rss2);
  ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm2 << " MB | RSS: " << rss2 << " MB");
  ROS_INFO_STREAM_NAMED(name_, "Current memory diff        - VM: " << vm2 - vm1 << " MB | RSS: " << rss2 - rss1 << " M"
                                                                                                                   "B");
  // Show database
  if (visualize_display_database_)
  {
    experience_setup_->getExperienceDB()->displayDatabase();
  }

  return true;
}

void CurieDemos::runRandomProblems()
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
    //imarker_start_->setToRandomState();
    //imarker_goal_->setToRandomState();
    moveit_start_ = imarker_start_->getRobotState();
    moveit_goal_ = imarker_goal_->getRobotState();

    // Visualize
    if (visualize_start_goal_states_)
      visualizeStartGoal();

    // Plan from start to goal
    plan(moveit_start_, moveit_goal_);

    // Main pause between planning instances - allows user to analyze
    ros::Duration(visualize_time_between_plans_).sleep();

    // Check if time to end loop
    if (!ros::ok())
      return;
  }

  // Save experience
  experience_setup_->doPostProcessing();

  // Console display
  experience_setup_->printLogs();

  // Logging
  // experience_setup_->saveDataLog(logging_file);
  // logging_file.flush();

  // Finishing up
  ROS_INFO_STREAM_NAMED(name_, "Saving experience db...");
  experience_setup_->saveIfChanged();

  // Stats
  // ROS_ERROR_STREAM_NAMED(name_, "Average solving time: " << (total_duration_ / total_runs_));
}

bool CurieDemos::plan(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
{
  // Setup -----------------------------------------------------------

  // Clear all planning data. This only includes data generated by motion plan computation.
  // Planner settings, start & goal states are not affected.
  experience_setup_->clear();

  // Convert MoveIt state to OMPL state
  space_->copyToOMPLState(ompl_start_, *start_state);
  space_->copyToOMPLState(ompl_goal_, *goal_state);

  // Convert the goal state to level 2
  const int level = 2;
  space_->setLevel(ompl_goal_, level);

  // Set the start and goal states
  experience_setup_->setStartAndGoalStates(ompl_start_, ompl_goal_);

  // Debug - this call is optional, but we put it in to get more output information
  // experience_setup_->print();

  // Cartesian -----------------------------------------------------------

  // Optionally create cartesian path
  generateRandCartesianPath();

  // Solve -----------------------------------------------------------

  // Create the termination condition
  double seconds = 1000;  // 0.1; //0.1;
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus solved = experience_setup_->solve(ptc);

  // Check for error
  if (!solved)
  {
    ROS_ERROR("No Solution Found");
    return false;
  }

  // Check for non-exact solution
  if (!experience_setup_->haveExactSolutionPath())
  {
    ROS_WARN_STREAM_NAMED(name_, "APPROXIMATE solution found from planner "
                                     << experience_setup_->getSolutionPlannerName());
  }
  else  // exact solution found
  {
    ROS_DEBUG_STREAM_NAMED(name_, "Exact solution found from planner " << experience_setup_->getSolutionPlannerName());

    // Display states on available solutions
    experience_setup_->printResultsInfo();
  }

  // Get solution
  og::PathGeometric path = experience_setup_->getSolutionPath();

  // Add start to solution
  path.prepend(ompl_start_);

  // Check/test the solution for errors
  checkOMPLPathSolution(path);

  // Clear Rviz
  visual_ompl3_->hideRobot();
  visual_ompl3_->deleteAllMarkers();

  // Visualize raw trajectory
  if (visualize_raw_trajectory_)
    visualizeRawTrajectory(path);

  // Smooth free-space components of trajectory
  std::size_t state_count = path.getStateCount();
  smoothFreeSpace(path);
  ROS_INFO_STREAM_NAMED(name_, "Smoothing removed: " << path.getStateCount() - state_count << " states");

  // Add more states between waypoints
  state_count = path.getStateCount();
  path.interpolate();
  ROS_INFO_STREAM_NAMED(name_, "Interpolation added: " << path.getStateCount() - state_count << " states");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.025;
  visual_ompl3_->convertPath(path, jmg_, traj, speed);

  // Check/test the solution for errors
  checkMoveItPathSolution(traj);

  // Visualize the trajectory
  if (visualize_interpolated_trajectory_)
  {
    ROS_INFO("Visualizing the interpolated trajectory");

    // Show trajectory line
    mvt::MoveItVisualToolsPtr visual_moveit3 = boost::dynamic_pointer_cast<mvt::MoveItVisualTools>(visual_ompl3_);
    visual_moveit3->publishTrajectoryLine(traj, ee_link_, rvt::RED);

    // Show trajectory
    const bool wait_for_trajectory = true;
    visual_moveit3->publishTrajectoryPath(traj, wait_for_trajectory);

    ros::Duration(1).sleep();
  }

  // Process the popularity
  experience_setup_->doPostProcessing();

  // Visualize the doneness
  std::cout << std::endl;

  return true;
}

void CurieDemos::visualizeRawTrajectory(og::PathGeometric& path)
{
  ROS_INFO("Visualizing non-interpolated trajectory");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.05;
  visual_ompl3_->convertPath(path, jmg_, traj, speed);

  // Show trajectory line
  mvt::MoveItVisualToolsPtr visual_moveit3 = boost::dynamic_pointer_cast<mvt::MoveItVisualTools>(visual_ompl3_);
  visual_moveit3->publishTrajectoryLine(traj, ee_link_, rvt::GREY);
  visual_moveit3->triggerBatchPublish();
}

void CurieDemos::smoothFreeSpace(og::PathGeometric& path)
{
  og::PathGeometric free_path_0(si_);
  og::PathGeometric cart_path_1(si_);
  og::PathGeometric free_path_2(si_);
  og::PathGeometric new_path(si_);

  // Separate path into level types
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    int level = space_->getLevel(path.getState(i));

    switch (level)
    {
      case 0:
        free_path_0.append(path.getState(i));
        break;
      case 1:
        cart_path_1.append(path.getState(i));
        break;
      case 2:
        free_path_2.append(path.getState(i));
        break;
      default:
        ROS_ERROR_STREAM_NAMED(name_, "Unknown level type " << level);
    }
  } // for

  // Smooth both free space plans
  simplifyPath(free_path_0);
  simplifyPath(free_path_2);

  // Combine paths back together
  new_path.append(free_path_0);
  new_path.append(cart_path_1);
  new_path.append(free_path_2);

  ROS_INFO_STREAM_NAMED(name_, "New path has " << new_path.getStateCount() << " states");

  // Copy back to original structure
  path = new_path;
}

bool CurieDemos::simplifyPath(og::PathGeometric& path)
{
  ros::Time start_time = ros::Time::now();
  std::size_t num_states = path.getStateCount();

  // Allow 1 second of simplification
  const ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(1.0);

  // Simplify
  experience_setup_->getPathSimplifier()->simplify(path, ptc);

  // Feedback
  double duration = (ros::Time::now() - start_time).toSec();
  OMPL_INFORM("SimpleSetup(ptc): Path simplification took %f seconds and changed from %d to %d states",
              duration, num_states, path.getStateCount());

  return true;
}

void CurieDemos::generateRandCartesianPath()
{
  // First cleanup previous cartesian paths
  experience_setup_->getExperienceDB()->cleanupTemporaryVerticies();

  // Get MoveIt path
  std::vector<moveit::core::RobotStatePtr> trajectory;
  if (!cart_path_planner_->getTrajectory(trajectory))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to get computed cartesian trajectory");
    exit(-1);
  }

  // Convert to OMPL path
  std::vector<ompl::base::State *> ompl_path;
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    ob::State *state = space_->allocState();
    space_->copyToOMPLState(state, *trajectory[i]);
    const std::size_t level = 1;
    space_->setLevel(state, level);

    ompl_path.push_back(state);
  }

  // Insert into graph
  std::cout << "adding path --------------------- " << std::endl;
  if (!experience_setup_->getExperienceDB()->addCartPath(ompl_path))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to add cartesian path");
    exit(-1);
  }
}

bool CurieDemos::checkOMPLPathSolution(og::PathGeometric& path)
{
  bool error = false;
  int current_level = 0;

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    int level = space_->getLevel(path.getState(i));

    // Check if start state is correct
    if (i == 0)
    {
      if (!space_->equalStates(path.getState(i), ompl_start_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Start state of path is not same as original problem");
        error = true;
      }

      if (level != 0)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Start state is not at level 0, instead " << level);
        error = true;
      }
    }

    // Check if goal state is correct
    if (i == path.getStateCount() - 1)
    {
      if (!space_->equalStates(path.getState(i), ompl_goal_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Goal state of path is not same as original problem");
        error = true;
      }

      if (level != 2)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Goal state is not at level 2, instead " << level);
        error = true;
      }
    }

    // Ensure that level is always increasing
    if (level < current_level)
    {
      ROS_ERROR_STREAM_NAMED(name_, "State decreased in level (" << level << ") from previous level of " << current_level);
      error = true;
    }
    current_level = level;

  } // for loop

  // Show more data if error
  if (error)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Showing data on path:");
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
      int level = space_->getLevel(path.getState(i));
      std::cout << "  - Path state " << i << " has level " << level << std::endl;
    }
  }

  return error;
}

bool CurieDemos::checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj)
{
  std::size_t state_count = traj->getWayPointCount();
  if (state_count < 3)
    ROS_WARN_STREAM_NAMED(name_, "checkMoveItPathSolution: Solution path has only " << state_count << " states");
  else
    ROS_INFO_STREAM_NAMED(name_, "checkMoveItPathSolution: Solution path has " << state_count << " states");

  //bool isPathValid(const robot_trajectory::RobotTrajectory &trajectory,
  //const std::string &group = "", bool verbose = false, std::vector<std::size_t> *invalid_index = NULL) const;

  std::vector<std::size_t> index;
  const bool verbose = true;
  if (!planning_scene_->isPathValid(*traj, "", verbose, &index))
  {
    if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
      ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
    else
    {
      // display error messages
      std::stringstream ss;
      for (std::size_t i = 0; i < index.size(); ++i)
        ss << index[i] << " ";
      ROS_ERROR_STREAM_NAMED(name_, "checkMoveItPathSolution: Computed path is not valid. Invalid states at index locations: [ "
                       << ss.str() << "] out of " << state_count << ". Explanations follow in command line.");

      // Call validity checks in verbose mode for the problematic states
      visualization_msgs::MarkerArray arr;
      for (std::size_t i = 0; i < index.size(); ++i)
      {
        /*
        // check validity with verbose on
        const robot_state::RobotState &robot_state = traj->getWayPoint(index[i]);
        planning_scene_->isStateValid(robot_state, request.path_constraints, request.group_name, true);

        // compute the contacts if any
        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.contacts = true;
        c_req.max_contacts = 10;
        c_req.max_contacts_per_pair = 3;
        c_req.verbose = false;
        planning_scene_->checkCollision(c_req, c_res, robot_state);
        */
        ROS_ERROR_STREAM_NAMED(name_, "checkMoveItPathSolution: TODO: show collision states in code " << i);
        /*
          if (c_res.contact_count > 0)
          {
          visualization_msgs::MarkerArray arr_i;
          collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene_->getPlanningFrame(),
          c_res.contacts);
          arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
          }
        */
      }
      ROS_ERROR_STREAM_NAMED(name_, "checkMoveItPathSolution: Completed listing of explanations for invalid states.");
      // if (!arr.markers.empty())
      //  contacts_publisher_.publish(arr);
    }
  }
  return true;
}

bool CurieDemos::getRandomState(moveit::core::RobotStatePtr &robot_state)
{
  static const std::size_t MAX_ATTEMPTS = 100;
  for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
  {
    robot_state->setToRandomPositions(jmg_);
    robot_state->update();

    // Error check
    bool check_verbose = false;
    if (planning_scene_->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                         // collision check, "" is everything
    {
      // ROS_DEBUG_STREAM_NAMED(name_, "Found valid random robot state after " << i << " attempts");
      return true;
    }
  }

  ROS_ERROR_STREAM_NAMED(name_, "Unable to find valid random robot state");
  exit(-1);
  return false;
}

void CurieDemos::loadVisualTools()
{
  // Note: this is in addition to the moveit_boilerplate visual_tools

  std::string namesp = nh_.getNamespace();
  visual_ompl1_.reset(
      new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/start_markers", robot_model_));
  visual_ompl1_->setPlanningSceneMonitor(planning_scene_monitor_);
  visual_ompl1_->loadRobotStatePub(namesp + "/start_state");
  visual_ompl1_->hideRobot();  // show that things have been reset
  visual_ompl1_->setManualSceneUpdating(true);
  visual_moveit_start_ = visual_ompl1_;

  visual_ompl2_.reset(
      new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/goal_markers", robot_model_));
  visual_ompl2_->setPlanningSceneMonitor(planning_scene_monitor_);
  visual_ompl2_->loadRobotStatePub(namesp + "/goal_state");
  visual_ompl2_->setManualSceneUpdating(true);
  visual_ompl2_->hideRobot();           // show that things have been reset
  visual_moveit_goal_ = visual_ompl2_;  // copy for use by Moveit

  visual_ompl3_.reset(
      new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/ompl_markers", robot_model_));
  visual_ompl3_->loadTrajectoryPub(namesp + "/display_trajectory");
  visual_ompl3_->setPlanningSceneMonitor(planning_scene_monitor_);
  visual_ompl3_->loadRobotStatePub(namesp + "/ompl_state");
  visual_ompl3_->setManualSceneUpdating(true);
  visual_ompl3_->hideRobot();  // show that things have been reset

  // Clear all previous markers
  visual_tools_->loadMarkerPub(true);
  visual_tools_->deleteAllMarkers();
  visual_tools_->triggerBatchPublish();
  ros::spinOnce();

  visual_ompl1_->loadMarkerPub(true);
  visual_ompl1_->deleteAllMarkers();
  visual_ompl1_->triggerBatchPublish();
  ros::spinOnce();

  visual_ompl2_->loadMarkerPub(true);
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  visual_ompl2_->deleteAllMarkers();
  visual_ompl2_->triggerBatchPublish();
  ros::spinOnce();

  visual_ompl3_->loadMarkerPub(true);
  visual_ompl3_->deleteAllMarkers();
  visual_ompl3_->triggerBatchPublish();
  ros::spinOnce();

  ros::Duration(1).sleep();
}

void CurieDemos::visualizeStartGoal()
{
  visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);
  visual_moveit_goal_->publishRobotState(moveit_goal_, rvt::ORANGE);

  // Show values and limits
  // std::cout << "Start: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_start_);
  // std::cout << "Goal: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_goal_);
}

void CurieDemos::testConnectionToGraphOfRandStates()
{
  ompl::base::State *random_state = space_->allocState();

  std::size_t successful_connections = 0;
  for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    ROS_INFO_STREAM_NAMED(name_, "Testing random state " << run_id);

    // Generate random state
    getRandomState(moveit_start_);

    // Visualize
    visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);

    // Convert to ompl
    space_->copyToOMPLState(random_state, *moveit_start_);

    // Test
    const ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(60.0);
    bool result = experience_setup_->getRetrieveRepairPlanner().canConnect(random_state, ptc);
    if (result)
      successful_connections++;

    ROS_ERROR_STREAM_NAMED(name_, "Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
  }

  space_->freeState(random_state);
}

bool CurieDemos::getFilePath(std::string &file_path, const std::string &database_name,
                                  const std::string &database_directory) const

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

  rootPath = rootPath / fs::path(database_directory);

  boost::system::error_code returnedError;
  fs::create_directories(rootPath, returnedError);

  if (returnedError)
  {
    // did not successfully create directories
    ROS_ERROR("Unable to create directory %s", database_directory.c_str());
    return false;
  }

  // directories successfully created, append the group name as the file name
  rootPath = rootPath / fs::path("bolt_" + database_name + "_database.ompl");
  file_path = rootPath.string();
  ROS_INFO_STREAM_NAMED("planning_context_manager", "Setting database to " << file_path);

  return true;
}

}  // namespace curie_demos
