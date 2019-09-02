cmake_minimum_required(VERSION 2.8.3)
project(mm_force_control)

# add_compile_options(-std=c++11)

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   geometry_msgs
#   mm_msgs
#   mm_kinematics
#   mm_math_util
#   message_runtime
# )
# find_package(cmake_modules REQUIRED)
# find_package(Eigen REQUIRED)

# Python setup
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
# TODO I don't really understand what's going on here
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES mm_force_control
#   CATKIN_DEPENDS geometry_msgs mm_msgs mm_kinematics mm_math_util
#   DEPENDS eigen
# )

###########
## Build ##
###########

# Header file locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Eigen_INCLUDE_DIRS}
#   ${mm_kinematics_INCLUDE_DIRS}
#   ${mm_math_util_INCLUDE_DIRS}
# )
#
# add_executable(mm_force_control
#   src/mm_force_control.cpp
# )
# target_link_libraries(mm_force_control
#   ${catkin_LIBRARIES}
# )
