cmake_minimum_required(VERSION 2.8.3)
project(mm_control)

add_compile_options(-std=c++11)

# Required for qpOASES
# add_compile_options(
#   -D__USE_LONG_INTEGERS__
#   -D__USE_LONG_FINTS__
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_runtime
  cmake_modules
  mm_kinematics
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Install Python module
catkin_python_setup()

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
  CATKIN_DEPENDS mm_kinematics
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
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Executables

add_executable(home
  src/nodes/home.cpp
)
target_link_libraries(home
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

