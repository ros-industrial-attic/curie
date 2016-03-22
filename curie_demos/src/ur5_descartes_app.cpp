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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Interface for planning a descartes path for UR5
*/

#include <curie_demos/ur5_descartes_app.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace curie_demos
{

UR5DescartesApp::UR5DescartesApp(moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
  : nh_("~")
  , name_("ur5_descartes_app")
  , visual_tools_(visual_tools)
{

}

UR5DescartesApp::~UR5DescartesApp()
{

}

void UR5DescartesApp::initDescartes()
{
  // Instantiating a robot model
  robot_model_ptr_.reset(new ur5_demo_descartes::UR5RobotModel());

  /*  Fill Code:
   * Goal:
   * - Initialize the "robot_model_ptr" variable by passing the required application parameters
   *    into its "initialize" method.
   * Hint:
   * - The config_ structure contains the variables needed by the robot model
   * - The "initialize" method takes the following arguments in this order
   *    a - robot description string
   *    b - group_name string.
   *    c - world_frame string
   *    d - tip_link string.
   */
  if(robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                  config_.group_name,
                                  config_.world_frame,
                                  config_.tip_link))
  {
    ROS_INFO_STREAM("Descartes Robot Model initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit(-1);
  }

  //Turn on collision checking.
  ROS_WARN_STREAM_NAMED(name_, "disabled collision checking");
  //robot_model_ptr_->setCheckCollisions(true);

  /*  Fill Code:
   * Goal:
   * - Initialize the Descartes path planner by calling "planner_.initialize(...)".
   * - Pass the robot_model_ptr_ created earlier into the initialize method and save the result
   *    into the "succeeded" variable.
   */
  bool succeeded = planner_.initialize(robot_model_ptr_);
  if(succeeded)
  {
    ROS_INFO_STREAM("Descartes Dense Planner initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to initialize Dense Planner");
    exit(-1);
  }

}

void UR5DescartesApp::loadParameters()
{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "group_name",config_.group_name);
  error += !rosparam_shortcuts::get(name_, rpnh, "tip_link",config_.tip_link);
  error += !rosparam_shortcuts::get(name_, rpnh, "base_link",config_.base_link);
  error += !rosparam_shortcuts::get(name_, rpnh, "world_frame",config_.world_frame);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/time_delay",config_.time_delay);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/foci_distance",config_.foci_distance);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/radius",config_.radius);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/num_points",config_.num_points);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/num_lemniscates",config_.num_lemniscates);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/center",config_.center);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory/seed_pose",config_.seed_pose);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualization/min_point_distance",config_.min_point_distance);
  error += !rosparam_shortcuts::get(name_, rpnh, "controller_joint_names",config_.joint_names);
  rosparam_shortcuts::shutdownIfError(name_, error);
}

void UR5DescartesApp::planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path)
{
  // planning robot path
  /*  Fill Code:
   * Goal:
   * - Call the "planner_.planPath(...)" method in order to plan a robot path from the trajectory.
   * - Save the result of the planPath(...) call into the succeeded variable in order to verify that
   *     a valid robot path was generated.
   * Hint:
   * - The "planner_.planPath(...)" can take the "input_traj" Trajectory as an input argument.
   */
  bool succeeded = planner_.planPath(input_traj);

  if (succeeded)
  {
    ROS_INFO_STREAM("Valid path was found");
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("Could not solve for a valid path");
    exit(-1);
  }

  // retrieving robot path
  /*  Fill Code:
   * Goal:
   * - Call the "planner_.getPath(...)" in order to retrieve the planned robot path.
   * - Save the result of the planPath(...) call into the succeeded variable in order to verify that
   *     a valid robot path was generated.
   * Hint:
   * - The "planner_.getPath(...)" can take the "output_path" variable as an output argument.
   */
  succeeded = planner_.getPath(output_path);

  if(!succeeded || output_path.empty())
  {
    ROS_ERROR_STREAM("Failed to retrieve robot path");
  }

}

moveit_msgs::RobotTrajectory UR5DescartesApp::runPath(const DescartesTrajectory& path)
{
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());
  std::vector<double> start_pose;

  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
  first_point_ptr->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // creating Moveit trajectory from Descartes Trajectory
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory);

  // sending robot path to server for execution
  return moveit_traj;
}

void UR5DescartesApp::publishPosesMarkers(const EigenSTL::vector_Affine3d& poses)
{
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;

  z_axes.type = y_axes.type = x_axes.type = visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action = visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id = x_axes.header.frame_id = config_.world_frame;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = AXIS_LINE_WIDTH;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration(0);
  line.header.frame_id = config_.world_frame;
  line.scale.x = AXIS_LINE_WIDTH;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve(2*poses.size());
  y_axes.points.reserve(2*poses.size());
  x_axes.points.reserve(2*poses.size());
  line.points.reserve(poses.size());
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Affine3d prev = poses[0];
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    const Eigen::Affine3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > config_.min_point_distance)
    {
      Eigen::Affine3d moved_along_x = pose * Eigen::Translation3d(AXIS_LINE_LENGHT,0,0);
      tf::pointEigenToMsg(moved_along_x.translation(),p_end);
      x_axes.points.push_back(p_start);
      x_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_y = pose * Eigen::Translation3d(0,AXIS_LINE_LENGHT,0);
      tf::pointEigenToMsg(moved_along_y.translation(),p_end);
      y_axes.points.push_back(p_start);
      y_axes.points.push_back(p_end);

      Eigen::Affine3d moved_along_z = pose * Eigen::Translation3d(0,0,AXIS_LINE_LENGHT);
      tf::pointEigenToMsg(moved_along_z.translation(),p_end);
      z_axes.points.push_back(p_start);
      z_axes.points.push_back(p_end);

      // saving previous
      prev = pose;
    }

    line.points.push_back(p_start);
  }

  markers_msg.markers.push_back(x_axes);
  markers_msg.markers.push_back(y_axes);
  markers_msg.markers.push_back(z_axes);
  markers_msg.markers.push_back(line);

  visual_tools_->publishMarkers(markers_msg);
  visual_tools_->triggerBatchPublish();
}

bool UR5DescartesApp::createLemniscateCurve(double foci_distance, double sphere_radius,
                                            int num_points, int num_lemniscates,const Eigen::Vector3d& sphere_center,
                                            EigenSTL::vector_Affine3d& poses)
{
  double a = foci_distance;
  double ro = sphere_radius;
  int npoints = num_points;
  int nlemns = num_lemniscates;
  Eigen::Vector3d offset(sphere_center[0],sphere_center[1],sphere_center[2]);
  Eigen::Vector3d unit_z,unit_y,unit_x;

  // checking parameters
  if(a <= 0 || ro <= 0 || npoints < 10 || nlemns < 1)
  {
    ROS_ERROR_STREAM("Invalid parameters for lemniscate curve were found");
    return false;
  }

  // generating polar angle values
  std::vector<double> theta(npoints);

  // interval 1 <-pi/4 , pi/4 >
  double d_theta = 2*M_PI_2/(npoints - 1);
  for(unsigned int i = 0; i < static_cast<std::size_t>(npoints)/2;i++)
  {
    theta[i] = -M_PI_4  + i * d_theta;
  }
  theta[0] = theta[0] + EPSILON;
  theta[npoints/2 - 1] = theta[npoints/2 - 1] - EPSILON;

  // interval 2 < 3*pi/4 , 5 * pi/4 >
  for(unsigned int i = 0; i < static_cast<std::size_t>(npoints)/2;i++)
  {
    theta[npoints/2 + i] = 3*M_PI_4  + i * d_theta;
  }
  theta[npoints/2] = theta[npoints/2] + EPSILON;
  theta[npoints - 1] = theta[npoints - 1] - EPSILON;

  // generating omega angle (lemniscate angle offset)
  std::vector<double> omega(nlemns);
  double d_omega = M_PI/(nlemns);
  for(unsigned int i = 0; i < static_cast<std::size_t>(nlemns);i++)
  {
     omega[i] = i*d_omega;
  }

  Eigen::Affine3d pose;
  double x,y,z,r,phi;

  poses.clear();
  poses.reserve(nlemns*npoints);
  for(unsigned int j = 0; j < static_cast<std::size_t>(nlemns);j++)
  {
    for(unsigned int i = 0 ; i < static_cast<std::size_t>(npoints);i++)
    {
      r = std::sqrt( std::pow(a,2) * std::cos(2*theta[i]) );
      phi = r < ro ? std::asin(r/ro):  (M_PI - std::asin((2*ro - r)/ro) );

      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
      z = ro * std::cos(phi);

      // determining orientation
      unit_z <<-x, -y , -z;
      unit_z.normalize();

      unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
      unit_y = (unit_z .cross(unit_x)).normalized();

      Eigen::Matrix3d rot;
      rot << unit_x(0),unit_y(0),unit_z(0)
         ,unit_x(1),unit_y(1),unit_z(1)
         ,unit_x(2),unit_y(2),unit_z(2);

      pose = Eigen::Translation3d(offset(0) + x,
                                  offset(1) + y,
                                  offset(2) + z) * rot;

      poses.push_back(pose);
    }
  }

  return true;
}

void UR5DescartesApp::fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                                      trajectory_msgs::JointTrajectory& out_traj)
{
  // Fill out information about our trajectory
  out_traj.header.stamp = ros::Time::now();
  out_traj.header.frame_id = config_.world_frame;
  out_traj.joint_names = config_.joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;

  // Loop through the trajectory
  for (unsigned int i = 0; i < in_traj.size(); i++)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;

    // getting joint position at current point
    const descartes_core::TrajectoryPtPtr& joint_point = in_traj[i];
    joint_point->getNominalJointPose(std::vector<double>(), *robot_model_ptr_, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += config_.time_delay;

    out_traj.points.push_back(pt);
  }

}

void UR5DescartesApp::generateTrajectory(DescartesTrajectory& traj)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;


  // generating trajectory using a lemniscate curve function.
  EigenSTL::vector_Affine3d poses;
  Eigen::Vector3d center(config_.center[0], config_.center[1], config_.center[2]);
  if(createLemniscateCurve(config_.foci_distance, config_.radius, config_.num_points,
                           config_.num_lemniscates, center, poses))
  {
    ROS_INFO_STREAM("Trajectory with "<<poses.size()<<" points was generated");
  }
  else
  {
    ROS_ERROR_STREAM("Trajectory generation failed");
    exit(-1);
  }

  // publishing trajectory poses for visualization
  publishPosesMarkers(poses);

  Eigen::Affine3d transform = visual_tools_->getSharedRobotState()->getGlobalLinkTransform("right_base_link");

  // creating descartes trajectory points
  traj.clear();
  traj.reserve(poses.size());
  for(unsigned int i = 0; i < poses.size(); i++)
  {
    //const Eigen::Affine3d& pose = poses[i];
    Eigen::Affine3d pose = transform * poses[i];

    // Get all possible solutions
    std::vector<std::vector<double> > joint_poses;
    if (!robot_model_ptr_->getAllIK(pose, joint_poses))
    {
      ROS_ERROR_STREAM_NAMED(name_, "getAllIK returned false");
      return;
    }

    // Error check
    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "getAllIK returned no solutions");
      return;
    }

    ROS_INFO_STREAM_NAMED(name_, "Found good joint pose");

    /*
     * Create AxialSymetricPt objects in order to define a trajectory cartesian point with
     *    rotational freedom about the tool's z axis.
     */
    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
        new descartes_trajectory::AxialSymmetricPt(pose, ORIENTATION_INCREMENT,
                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,
                                                    descartes_core::TimingConstraint(0.5) ) );

    // saving points into trajectory
    traj.push_back(pt);
  }

}


} /* namespace curie_demos */
