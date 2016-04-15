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

#ifndef CURIE_DEMOS_CURIE_DEMOS_H
#define CURIE_DEMOS_CURIE_DEMOS_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <curie_demos/moveit_base.h>
#include <moveit_ompl/model_based_state_space.h>

// OMPL
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/bolt/Bolt.h>
#include <ompl_visual_tools/ompl_visual_tools.h>

// this package
#include <curie_demos/process_mem_usage.h>
#include <curie_demos/state_validity_checker.h>
#include <curie_demos/cart_path_planner.h>
#include <curie_demos/imarker_robot_state.h>

namespace mo = moveit_ompl;

namespace curie_demos
{
class CurieDemos : public curie_demos::MoveItBase
{
public:

  /** \brief Constructor */
  CurieDemos();

  /** \brief Destructor */
  ~CurieDemos();

  /** \brief Load the basic planning context components */
  bool loadOMPL();

  /** \brief Generate states for testing */
  void testConnectionToGraphOfRandStates();

  void runProblems();

  bool plan(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state);

  /** \brief Create multiple dummy cartesian paths */
  void generateRandCartesianPath();

  bool checkOMPLPathSolution(og::PathGeometric& path);
  bool checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj);

  bool getRandomState(moveit::core::RobotStatePtr &robot_state);

  /**
   * \brief Dump the entire database contents to Rviz
   */
  void displayDatabase()
  {
    bolt_setup_->getDenseDB()->displayDatabase();
  }

  bool getShouldSkipSolving()
  {
    return skip_solving_;
  }

  /**
   * \brief Clear all markers displayed in Rviz
   */
  void deleteAllMarkers(bool clearDatabase = true);

  void loadVisualTools();

  void visualizeStartGoal();

  void visualizeRawTrajectory(og::PathGeometric& path);

  /** \brief Smooths the free space components of a trajectory, but not the cartesian components */
  void smoothFreeSpace(og::PathGeometric& path);

  bool simplifyPath(og::PathGeometric& path);

  // --------------------------------------------------------

  // A shared node handle
  ros::NodeHandle nh_;

  // The short name of this class
  std::string name_;

  // For visualizing things in rviz
  ompl_visual_tools::OmplVisualToolsPtr viz1_;
  ompl_visual_tools::OmplVisualToolsPtr viz2_;
  ompl_visual_tools::OmplVisualToolsPtr viz3_;
  ompl_visual_tools::OmplVisualToolsPtr viz4_;
  ompl_visual_tools::OmplVisualToolsPtr viz5_;
  ompl_visual_tools::OmplVisualToolsPtr viz6_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_moveit_start_;  // Clone of ompl1
  moveit_visual_tools::MoveItVisualToolsPtr visual_moveit_goal_;   // Clone of ompl2

  // Robot states
  moveit::core::RobotStatePtr moveit_start_;
  moveit::core::RobotStatePtr moveit_goal_;
  ob::State* ompl_start_;
  ob::State* ompl_goal_;

  // Planning groups
  std::string planning_group_name_;
  moveit::core::JointModelGroup *jmg_;
  moveit::core::LinkModel *ee_link_;

  // Operation settings
  bool auto_run_;
  std::string experience_planner_;
  std::size_t planning_runs_;
  bool skip_solving_;
  bool use_task_planning_;
  int post_processing_interval_;

  // Debug and display preferences
  bool visualize_display_database_;
  bool visualize_interpolated_traj_;
  //bool visualize_raw_trajectory_;
  bool visualize_grid_generation_;
  bool visualize_start_goal_states_;
  //bool visualize_astar_;
  bool visualize_cart_neighbors_;
  bool visualize_cart_path_;
  double visualize_time_between_plans_;
  bool visualize_database_every_plan_;
  bool debug_print_trajectory_;

  // Configuration space
  ompl::base::SpaceInformationPtr si_;

  moveit_ompl::ModelBasedStateSpacePtr space_;
  ompl::tools::bolt::BoltPtr bolt_setup_;

  // Average planning time
  double total_duration_;
  std::size_t total_runs_;
  std::size_t total_failures_;

  // Create constrained paths
  CartPathPlannerPtr cart_path_planner_;

  // Interactive markers
  IMarkerRobotStatePtr imarker_start_;
  IMarkerRobotStatePtr imarker_goal_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CurieDemos> CurieDemosPtr;
typedef boost::shared_ptr<const CurieDemos> CurieDemosConstPtr;

}  // namespace curie_demos

#endif  // CURIE_DEMOS_CURIE_DEMOS_H
