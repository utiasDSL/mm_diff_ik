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
  cmake_modules
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
  CATKIN_DEPENDS mm_msgs mm_kinematics mm_math_util
)

###########
## Build ##
###########

# Header files

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mm_kinematics_INCLUDE_DIRS}
  ${mm_math_util_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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

## analyse_optimizer
add_executable(analyse_optimizer
  src/nodes/analyse_optimizer.cpp
  src/pose_control/pose_error.cpp src/pose_control/pose_error_jac.cpp
  src/pose_control/optimizer.cpp
  src/pose_control/trajectory.cpp
)
target_link_libraries(analyse_optimizer
  ${catkin_LIBRARIES}
  /usr/local/lib/libqpOASES.so
)

# Tests
# catkin_add_executable_with_gtest(test_optimize
#   test/test_optimize.cpp
#   src/pose_control/optimizer.cpp
#   src/pose_control/trajectory.cpp
#   src/pose_control/pose_error.cpp
#   src/pose_control/pose_error_jac.cpp
# )
# target_link_libraries(test_optimize
#   ${catkin_LIBRARIES}
#   /usr/local/lib/libqpOASES.so
# )
