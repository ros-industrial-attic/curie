# Hilgendorf Demos

Description: Demonstrate dual arm manipulation using a combination of free space and Cartesian planning

Features:

 - TODO

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder in collaboration with ROS Industrial, Southwest Research Institute, and the National Institute of Standards and Technology.

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/hilgendorf_demos.svg)](https://travis-ci.org/davetcoleman/hilgendorf_demos) Travis - Continuous Integration
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__hilgendorf_demos__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__hilgendorf_demos__ubuntu_trusty__source/) ROS Buildfarm - Trusty Devel Source Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__hilgendorf_demos__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__hilgendorf_demos__ubuntu_trusty_amd64__binary/) ROS Buildfarm - AMD64 Trusty Debian Build

![](resources/screenshot.png)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-jade-hilgendorf-demos

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro jade

## Code API

> Note: this package has not been released yet

See [Class Reference](http://docs.ros.org/jade/api/hilgendorf_demos/html/)

## Usage

Start Rviz:

    roslaunch hilgendorf_moveit_demos hilgendorf_visualize.launch

Start simulated ros_control:

    roslaunch hilgendorf_moveit_demos hilgendorf_simulation.launch

Run example demo:

    roslaunch hilgendorf_moveit_demos hilgendorf_demo.launch

## Distance between poses

Test code

    rosrun tf_keyboard_cal tf_interactive_marker.py world thing 0 0 0 0 0 0 1

    rosrun hilgendorf_moveit_demos test_pose_distance
## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
