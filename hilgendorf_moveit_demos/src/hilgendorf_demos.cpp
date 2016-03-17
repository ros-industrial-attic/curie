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
#include <hilgendorf_moveit_demos/hilgendorf_demos.h>

namespace hilgendorf_moveit_demos
{
HilgendorfDemos::HilgendorfDemos()
  : MoveItBase(), nh_("~"), name_("hilgendorf_demos"), total_duration_(0.0), total_runs_(0), total_failures_(0)

{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
  error += !rosparam_shortcuts::get(name_, rpnh, "sparse_delta", sparse_delta_);
  error += !rosparam_shortcuts::get(name_, rpnh, "save_database", save_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "skip_solving", skip_solving_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  // Visualize
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/playback_trajectory", visualize_playback_trajectory_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/grid_generation", visualize_grid_generation_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/astar", visualize_astar_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/time_between_plans", visualize_time_between_plans_);
  // Debug
  error += !rosparam_shortcuts::get(name_, rpnh, "debug/print_trajectory", debug_print_trajectory_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Initialize MoveIt base
  MoveItBase::init(nh_);

  // Load 2 more visual tools
  loadVisualTools();
  ros::Duration(1).sleep();

  // Load 2 more robot states
  start_state_.reset(new moveit::core::RobotState(*current_state_));
  goal_state_.reset(new moveit::core::RobotState(*current_state_));

  // Get the two arms jmg
  jmg_ = robot_model_->getJointModelGroup(planning_group_name_);
  ee_link_ = robot_model_->getLinkModel("right_gripper_target");

  double vm1, rss1;
  process_mem_usage(vm1, rss1);
  ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");

  // Create cartesian planner
  cart_path_planner_.reset(new CartPathPlanner(this));

  // Create start/goal state imarker
  imarker_start_.reset(new IMarkerRobotState(planning_scene_monitor_, "start", jmg_, rvt::GREEN));
  imarker_goal_.reset(new IMarkerRobotState(planning_scene_monitor_, "goal", jmg_, rvt::ORANGE));

  ros::spin();
  exit(0);

  // Load planning
  if (!loadOMPL())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
    return;
  }

  ROS_INFO_STREAM_NAMED(name_, "HilgendorfDemos Ready.");
}

bool HilgendorfDemos::loadOMPL()
{
  moveit_ompl::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, jmg_);

  // Construct the state space we are planning in
  space_.reset(new moveit_ompl::ModelBasedStateSpace(mbss_spec));
  experience_setup_.reset(new ompl::tools::Bolt(space_));
  si_ = experience_setup_->getSpaceInformation();

  // Add custom distance function
  // space_->setDistanceFunction(boost::bind(&HilgendorfDemos::customDistanceFunction, this, _1, _2));

  // Setup base OMPL stuff
  experience_setup_->setup();
  assert(si_->isSetup());

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

  // Set planner settings
  experience_setup_->getExperienceDB()->visualizeAstar_ = visualize_astar_;
  experience_setup_->getExperienceDB()->sparseDelta_ = sparse_delta_;
  experience_setup_->getExperienceDB()->visualizeGridGeneration_ = visualize_grid_generation_;
  experience_setup_->getExperienceDB()->setSavingEnabled(save_database_);

  // Auto setup parameters (optional actually)
  // experience_setup_->enablePlanningFromRecall(use_recall_);
  // experience_setup_->enablePlanningFromScratch(use_scratch_);

  // Set state validity checking for this space
  experience_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(
      new moveit_ompl::StateValidityChecker(planning_group_name_, si_, *current_state_, planning_scene_, space_)));

  // The interval in which obstacles are checked for between states
  // seems that it default to 0.01 but doesn't do a good job at that level
  si_->setStateValidityCheckingResolution(0.005);

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
  // experience_setup_->getExperienceDB()->displayDatabase();

  return true;
}

void HilgendorfDemos::testRandomStates()
{
  ompl::base::State *random_state = space_->allocState();

  std::size_t successful_connections = 0;
  for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    ROS_INFO_STREAM_NAMED(name_, "Testing random state " << run_id);

    // Generate random state
    getRandomState(start_state_);

    // Visualize
    visual_moveit_start_->publishRobotState(start_state_, rvt::GREEN);

    // Convert to ompl
    space_->copyToOMPLState(random_state, *start_state_);

    // Test
    const ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(60.0);
    bool result = experience_setup_->getRetrieveRepairPlanner().canConnect(random_state, ptc);
    if (result)
      successful_connections++;

    ROS_ERROR_STREAM_NAMED(name_, "Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
  }

  space_->freeState(random_state);
}

void HilgendorfDemos::runRandomProblems()
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
    //getRandomState(start_state_);
    //getRandomState(goal_state_);
    start_state_ = imarker_start_->getRobotState();
    goal_state_ = imarker_goal_->getRobotState();

    // Visualize
    if (visualize_start_goal_states_)
      visualizeStartGoal();

    // Plan from start to goal
    plan(start_state_, goal_state_);

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

bool HilgendorfDemos::plan(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state)
{
  // Setup -----------------------------------------------------------

  // Clear all planning data. This only includes data generated by motion plan computation.
  // Planner settings, start & goal states are not affected.
  experience_setup_->clear();

  // Create start and goal space
  ob::ScopedState<> start(space_);
  ob::ScopedState<> goal(space_);

  // Convert MoveIt state to OMPL state
  space_->copyToOMPLState(start.get(), *start_state);
  space_->copyToOMPLState(goal.get(), *goal_state);

  // Set the start and goal states
  experience_setup_->setStartAndGoalStates(start, goal);

  // Debug - this call is optional, but we put it in to get more output information
  // experience_setup_->print();

  // Cartesian -----------------------------------------------------------

  // Optionally create cartesian path
  generateRandCartesianPath();

  ROS_WARN_STREAM_NAMED(name_, "Exiting for now");
  return false;


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

  // Visualize the trajectory
  if (visualize_playback_trajectory_)
  {
    ROS_INFO("Visualizing the trajectory");

    // Clear Rviz
    visual_ompl3_->hideRobot();
    visual_ompl3_->deleteAllMarkers();

    // Show trajectory
    const bool wait_for_trajectory = true;
    visual_ompl3_->loadTrajectoryPub("/hilgendorf/display_trajectory");
    robot_trajectory::RobotTrajectoryPtr traj =
        visual_ompl3_->publishRobotPath(experience_setup_->getSolutionPath(), jmg_, wait_for_trajectory);

    // Show trajectory line
    const bool clear_all_markers = true;
    mvt::MoveItVisualToolsPtr visual_moveit3 = boost::dynamic_pointer_cast<mvt::MoveItVisualTools>(visual_ompl3_);
    visual_moveit3->publishTrajectoryLine(traj, ee_link_, rvt::RED, clear_all_markers);
    visual_moveit3->triggerBatchPublish();
    ros::Duration(2).sleep();
  }

  // Process the popularity
  experience_setup_->doPostProcessing();

  // Visualize the doneness
  std::cout << std::endl;

  return true;
}

/** \brief Create multiple dummy cartesian paths */
void HilgendorfDemos::generateRandCartesianPath()
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
  std::vector<ompl::base::State*> ompl_path;
  for (std::size_t i = 0; i < trajectory.size(); ++i)
  {
    ob::State *state = space_->allocState();
    space_->copyToOMPLState(state, *trajectory[i]);

    // Adjust the task level information
    mo::ModelBasedStateSpace::StateType *mb_state = static_cast<mo::ModelBasedStateSpace::StateType*>(state);
    mb_state->level = 1;

    ompl_path.push_back(state);
  }

  // Insert into graph
  std::cout << "adding path --------------------- " << std::endl;
  experience_setup_->getExperienceDB()->addCartPath(ompl_path);
}

bool HilgendorfDemos::checkPathSolution(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                        const planning_interface::MotionPlanRequest &request,
                                        planning_interface::MotionPlanResponse &result)
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
        const robot_state::RobotState &robot_state = result.trajectory_->getWayPoint(index[i]);
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

bool HilgendorfDemos::getRandomState(moveit::core::RobotStatePtr &robot_state)
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

void HilgendorfDemos::loadVisualTools()
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
  visual_ompl2_->hideRobot();    // show that things have been reset
  visual_moveit_goal_ = visual_ompl2_;  // copy for use by Moveit

  visual_ompl3_.reset(
      new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), namesp + "/ompl_markers", robot_model_));
  visual_ompl3_->setPlanningSceneMonitor(planning_scene_monitor_);
  visual_ompl3_->loadRobotStatePub(namesp + "/ompl_state");
  visual_ompl3_->setManualSceneUpdating(true);
  visual_ompl3_->hideRobot();  // show that things have been reset

  // Clear all previous markers
  ROS_ERROR_STREAM_NAMED(name_, "Deleting all markers");
  ros::spinOnce();
  visual_tools_->loadMarkerPub();
  visual_tools_->deleteAllMarkers();
  visual_tools_->triggerBatchPublish();
  ros::spinOnce();

  ros::spinOnce();
  visual_ompl1_->loadMarkerPub(true);
  visual_ompl1_->deleteAllMarkers();
  visual_ompl1_->triggerBatchPublish();
  ros::spinOnce();

  visual_ompl2_->loadMarkerPub();
  visual_ompl2_->deleteAllMarkers();
  visual_ompl2_->triggerBatchPublish();
  ros::spinOnce();
  visual_ompl3_->loadMarkerPub();
  visual_ompl3_->deleteAllMarkers();
  visual_ompl3_->triggerBatchPublish();
  ros::spinOnce();
}

void HilgendorfDemos::visualizeStartGoal()
{
  visual_moveit_start_->publishRobotState(start_state_, rvt::GREEN);
  visual_moveit_goal_->publishRobotState(goal_state_, rvt::ORANGE);

  // Show values and limits
  // std::cout << "Start: " << std::endl;
  // visual_moveit_start_->showJointLimits(start_state_);
  // std::cout << "Goal: " << std::endl;
  // visual_moveit_start_->showJointLimits(goal_state_);
}

/*
double customDistanceFunction(const ompl::base::State *state1, const ompl::base::State *state2)
{
  std::cout << "no one should call this " << std::endl;
  exit(-1);

  space_->copyToRobotState(*from_state_, state1);
  space_->copyToRobotState(*to_state_, state2);

  const Eigen::Affine3d from_pose = from_state_->getGlobalLinkTransform(ee_link_);
  const Eigen::Affine3d to_pose = to_state_->getGlobalLinkTransform(ee_link_);

  return getPoseDistance(from_pose, to_pose);
}

double getPoseDistance(const Eigen::Affine3d &from_pose, const Eigen::Affine3d &to_pose)
{
  const double translation_dist = (from_pose.translation() - to_pose.translation()).norm();
  // const double distance_wrist_to_finger = 0.25; // meter

  const Eigen::Quaterniond from(from_pose.rotation());
  const Eigen::Quaterniond to(to_pose.rotation());

  // std::cout << "From: " << from.x() << ", " << from.y() << ", " << from.z() << ", " << from.w() << std::endl;
  // std::cout << "To: " << to.x() << ", " << to.y() << ", " << to.z() << ", " << to.w() << std::endl;

  double rotational_dist = arcLength(from, to);  // * distance_wrist_to_finger;

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
*/

/**
 * \brief Creates a directory names *database_direction* in the user's *home* folder, and inside that creates a file
 *        named *database_name.ompl*
 * \param file_path - result to generate
 * \param database_name - name of file to create
 * \param database_directory - name of folder to save in user directory
 * \return true on success
 */
bool HilgendorfDemos::getFilePath(std::string &file_path, const std::string &database_name,
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

}  // namespace hilgendorf_moveit_demos
