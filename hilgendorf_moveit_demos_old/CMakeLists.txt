cmake_minimum_required(VERSION 2.8.3)
project(hilgendorf_moveit_demos)

find_package(catkin REQUIRED COMPONENTS
  moveit_humanoid
  moveit_core
  ompl_visual_tools
  moveit_ompl
  random_numbers
  moveit_core
  moveit_ros_planning
  pluginlib
  moveit_visual_tools
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_humanoid
    moveit_core
    ompl_visual_tools
    moveit_ompl
    random_numbers
    moveit_core
    moveit_ros_planning
    moveit_visual_tools
  DEPENDS
    Eigen
)

###########
## Build ##
###########
include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(hilgendorf_demos src/hilgendorf_demos.cpp)
target_link_libraries(hilgendorf_demos
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(
  TARGETS
    hilgendorf_demos
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
