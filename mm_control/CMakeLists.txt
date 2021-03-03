cmake_minimum_required(VERSION 2.8.3)
project(mm_control)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_runtime
  cmake_modules
  mm_kinematics
  mm_msgs
  mm_math_util
  trajectory_msgs
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mm_kinematics mm_msgs mm_math_util trajectory_msgs
)

# Headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mm_kinematics_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/control.cpp
  src/cartesian/control.cpp
  src/cartesian/trajectory.cpp
  src/cartesian/spline.cpp
  src/joint/control.cpp
  src/joint/trajectory.cpp
  src/joint/spline.cpp
  src/joint/point.cpp
  src/joint/pd.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Executables

## joint control
add_executable(joint_control
  src/nodes/joint_control.cpp
)
target_link_libraries(joint_control
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## joint_state_mux
add_executable(joint_state_mux
  src/nodes/joint_state_mux_node.cpp
)
target_link_libraries(joint_state_mux
  ${catkin_LIBRARIES}
)
