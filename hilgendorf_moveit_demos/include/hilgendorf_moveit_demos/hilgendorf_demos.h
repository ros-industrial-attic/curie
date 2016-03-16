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

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_boilerplate/moveit_base.h>
#include <moveit_ompl/model_based_state_space.h>

// OMPL
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/bolt/Bolt.h>
#include <ompl_visual_tools/ompl_visual_tools.h>

// this package
#include <hilgendorf_moveit_demos/process_mem_usage.h>
#include <hilgendorf_moveit_demos/state_validity_checker.h>
#include <hilgendorf_moveit_demos/cart_path_planner.h>

namespace hilgendorf_moveit_demos
{
class HilgendorfDemos : public moveit_boilerplate::MoveItBase
{
public:
  /**
   * \brief Constructor
   */
  HilgendorfDemos();

  bool loadOMPL();

  void testRandomStates();

  void runRandomProblems();

  bool plan(robot_state::RobotStatePtr start_state, robot_state::RobotStatePtr goal_state);

  /** \brief Create multiple dummy cartesian paths */
  void generateRandCartesianPath();

  bool checkPathSolution(const planning_scene::PlanningSceneConstPtr &planning_scene,
                         const planning_interface::MotionPlanRequest &request,
                         planning_interface::MotionPlanResponse &result);

  bool getRandomState(moveit::core::RobotStatePtr &robot_state);

  void loadVisualTools();

  void visualizeStartGoal();

  /**
   * \brief Creates a directory names *database_direction* in the user's *home* folder, and inside that creates a file
   *        named *database_name.ompl*
   * \param file_path - result to generate
   * \param database_name - name of file to create
   * \param database_directory - name of folder to save in user directory
   * \return true on success
   */
  bool getFilePath(std::string &file_path, const std::string &database_name,
                   const std::string &database_directory) const;

  // --------------------------------------------------------

  // A shared node handle
  ros::NodeHandle nh_;

  // The short name of this class
  std::string name_;

  // For visualizing things in rviz
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl1_;
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl2_;
  ompl_visual_tools::OmplVisualToolsPtr visual_ompl3_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_start_;  // Clone of ompl1
  moveit_visual_tools::MoveItVisualToolsPtr visual_goal_;   // Clone of ompl2

  // Robot states
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr goal_state_;

  // Planning groups
  std::string planning_group_name_;
  moveit::core::JointModelGroup *jmg_;
  moveit::core::LinkModel *ee_link_;

  // Operation settings
  std::string experience_planner_;
  std::size_t planning_runs_;
  double sparse_delta_;
  bool save_database_;
  bool skip_solving_;

  // Debug and display preferences
  bool visualize_playback_trajectory_;
  bool visualize_grid_generation_;
  bool visualize_start_goal_states_;
  bool visualize_astar_;
  double visualize_time_between_plans_;
  bool debug_print_trajectory_;

  // Configuration space
  ompl::base::SpaceInformationPtr si_;

  moveit_ompl::ModelBasedStateSpacePtr space_;
  ompl::tools::BoltPtr experience_setup_;

  // Average planning time
  double total_duration_;
  std::size_t total_runs_;
  std::size_t total_failures_;

  // Create constrained paths
  CartPathPlannerPtr cart_path_planner_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<HilgendorfDemos> HilgendorfDemosPtr;
typedef boost::shared_ptr<const HilgendorfDemos> HilgendorfDemosConstPtr;

}  // namespace hilgendorf_moveit_demos

#endif  // HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
