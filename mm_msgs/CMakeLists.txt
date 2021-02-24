cmake_minimum_required(VERSION 2.8.3)
project(mm_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PoseTrajectoryPoint.msg
  PoseTrajectory.msg
  PoseControlState.msg
  OptimizationState.msg
  ForceControlState.msg
  ForceInfo.msg
  WrenchInfo.msg
  Obstacle.msg
  Obstacles.msg
  CartesianState.msg
  CartesianControllerState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mm_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Create C++ library
add_library(${PROJECT_NAME}
  src/conversions.cpp
)

# required to tell cmake that message headers must be generated first
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
