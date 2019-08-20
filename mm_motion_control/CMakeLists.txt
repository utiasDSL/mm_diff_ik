cmake_minimum_required(VERSION 2.8.3)
project(mm_motion_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mm_msgs
  mm_kinematics
  message_runtime
)
find_package(cmake_modules REQUIRED) # required for eigen
find_package(Eigen REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  CATKIN_DEPENDS mm_msgs mm_kinematics
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
  /usr/local/include/eigen-quadprog
)

# Executables

add_executable(mm_control_node
  src/mm_control_node.cpp src/control.cpp
)
target_link_libraries(mm_control_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libeigen-quadprog.so
)

add_executable(joint_state_mux
  src/joint_state_mux.cpp
)
target_link_libraries(joint_state_mux
  ${catkin_LIBRARIES}
)

# Tests

catkin_add_executable_with_gtest(test_interp
  test/test_interp.cpp
)

catkin_add_executable_with_gtest(test_optimize
  test/test_optimize.cpp
)
target_link_libraries(test_optimize
  ${catkin_LIBRARIES}
  /usr/local/lib/libeigen-quadprog.so
)
