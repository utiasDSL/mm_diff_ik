cmake_minimum_required(VERSION 2.8.3)
project(mm_control)

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
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  cmake_modules
  mm_kinematics
  mm_msgs
  mm_math_util
  mm_optimization
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
  ${mm_math_util_INCLUDE_DIRS}
  ${mm_optimization_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/qpOASES
)

# Library
add_library(${PROJECT_NAME}
  src/control.cpp
  src/cartesian/point.cpp
  src/cartesian/spline.cpp
  src/cartesian/trajectory.cpp
  src/cartesian/control.cpp
  src/cartesian/diffik.cpp
  src/cartesian/mpc.cpp
  src/joint/point.cpp
  src/joint/spline.cpp
  src/joint/trajectory.cpp
  src/joint/control.cpp
  src/joint/pd.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  /usr/local/lib/libqpOASES.so
)

# Executables

# Differential IK
add_executable(diff_ik
  src/nodes/diff_ik.cpp
)
target_link_libraries(diff_ik
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

## Cartesian MPC
add_executable(mpc
  src/nodes/mpc.cpp
)
target_link_libraries(mpc
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

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
  src/nodes/joint_state_mux.cpp
)
target_link_libraries(joint_state_mux
  ${catkin_LIBRARIES}
)
