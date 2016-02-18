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
   Desc:
*/

#ifndef HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
#define HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// MoveIt
#include <moveit_boilerplate/moveit_base.h>

namespace hilgendorf_moveit_demos
{

class HilgendorfDemos : public moveit_boilerplate::MoveItBase
{
public:

  /**
   * \brief Constructor
   */
  HilgendorfDemos()
    : MoveItBase()
  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    //error += !rosparam_shortcuts::get(name_, rpnh, "joint_state_topic", joint_state_topic);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Initialize MoveIt base
    init(nh_);

    ROS_INFO_STREAM_NAMED(name_,"HilgendorfDemos Ready.");
  }

private:

  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "hilgendorf_demos";

  // A shared node handle
  ros::NodeHandle nh_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<HilgendorfDemos> HilgendorfDemosPtr;
typedef boost::shared_ptr<const HilgendorfDemos> HilgendorfDemosConstPtr;

} // namespace hilgendorf_moveit_demos

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

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif  // HILGENDORF_MOVEIT_DEMOS_HILGENDORF_DEMOS_H
