cmake_minimum_required(VERSION 2.8.3)
project(mm_optimization)

add_compile_options(-std=c++11)

# Required for qpOASES
add_compile_options(
  -D__USE_LONG_INTEGERS__
  -D__USE_LONG_FINTS__
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

# Headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/qpOASES
)

# Library
add_library(${PROJECT_NAME}
  src/qpoases.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  /usr/local/lib/libqpOASES.so
)
