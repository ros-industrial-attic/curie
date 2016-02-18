cmake_minimum_required(VERSION 2.8.3)
project(hilgendorf_moveit_demos)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  moveit_boilerplate
  moveit_core
  moveit_visual_tools
  rosparam_shortcuts
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_boilerplate
    moveit_core
    moveit_visual_tools
    roscpp
    rosparam_shortcuts
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# # Library
# add_library(${PROJECT_NAME}
#   src/MY_LIBRARY.cpp
# )
# # Rename C++ executable without namespace
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# Main program
add_executable(${PROJECT_NAME}_hilgendorf_demos
  src/hilgendorg_demos.cpp
)
# Rename C++ executable without namespace
set_target_properties(${PROJECT_NAME}_hilgendorf_demos
  PROPERTIES OUTPUT_NAME hilgendorf_demos PREFIX "")
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_hilgendorf_demos
  #${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_hilgendorf_demos
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# Mark roslaunch files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
# install(FILES config
#   FILES_MATCHING PATTERN "*.yaml"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )