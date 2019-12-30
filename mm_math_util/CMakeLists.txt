cmake_minimum_required(VERSION 2.8.3)
project(mm_math_util)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(cmake_modules REQUIRED)
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
  INCLUDE_DIRS include
  #  LIBRARIES mm_math_util
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Create C++ library
# add_library(${PROJECT_NAME}
#   src/kinematics.cpp
# )
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )


# Tests
catkin_add_executable_with_gtest(test_differentiation
  test/test_differentiation.cpp
)
catkin_add_executable_with_gtest(test_wrap
  test/test_wrap.cpp
)
catkin_add_executable_with_gtest(test_interp
  test/test_interp.cpp
)
