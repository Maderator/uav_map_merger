cmake_minimum_required(VERSION 2.8.3)
project(relative_pose_estimator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_lib
  geometry_msgs
  nav_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RelativePoseEstimator
  CATKIN_DEPENDS roscpp geometry_msgs mrs_lib nav_msgs
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

## Declare a C++ library
add_library(RelativePoseEstimator 
  src/RelativePoseEstimator.cpp 
  src/transformations.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(RelativePoseEstimator
  ${catkin_LIBRARIES}
  )
