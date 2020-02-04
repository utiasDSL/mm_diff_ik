cmake_minimum_required(VERSION 2.8.3)
project(mm_motion_control)

add_compile_options(-std=c++11)

# Required for qpOASES
add_compile_options(
  -D__USE_LONG_INTEGERS__
  -D__USE_LONG_FINTS__
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mm_msgs
  mm_kinematics
  mm_math_util
  message_runtime
)
find_package(cmake_modules REQUIRED) # required for eigen
find_package(Eigen REQUIRED)


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
  CATKIN_DEPENDS mm_msgs mm_kinematics mm_math_util
  DEPENDS eigen
)

###########
## Build ##
###########

# Header files

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${mm_kinematics_INCLUDE_DIRS}
  ${mm_math_util_INCLUDE_DIRS}
  /usr/local/include/eigen-quadprog
  /usr/local/include/qpOASES
)

# Executables

## mm_control node
add_executable(mm_control_node
  src/nodes/pose_control_node.cpp
  src/pose_control/controller.cpp src/pose_control/manager.cpp
  src/pose_control/pose_error.cpp src/pose_control/pose_error_jac.cpp
  src/pose_control/optimizer.cpp src/pose_control/mpc_optimizer.cpp
  src/pose_control/trajectory.cpp
)
target_link_libraries(mm_control_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libqpOASES.so
)

## joint_state_mux
add_executable(joint_state_mux
  src/nodes/joint_state_mux_node.cpp
)
target_link_libraries(joint_state_mux
  ${catkin_LIBRARIES}
)

## home
add_executable(home
  src/nodes/go_home_node.cpp
  src/joint_control/controller.cpp src/joint_control/manager.cpp
)
target_link_libraries(home
  ${catkin_LIBRARIES}
)

# add_executable(pose_publisher
#   src/pose_publisher.cpp
# )
# target_link_libraries(pose_publisher
#   ${catkin_LIBRARIES}
# )

# Tests
catkin_add_executable_with_gtest(test_optimize
  test/test_optimize.cpp
  src/pose_control/optimizer.cpp
  src/pose_control/trajectory.cpp
  src/pose_control/pose_error.cpp
  src/pose_control/pose_error_jac.cpp
)
target_link_libraries(test_optimize
  ${catkin_LIBRARIES}
  /usr/local/lib/libqpOASES.so
)
