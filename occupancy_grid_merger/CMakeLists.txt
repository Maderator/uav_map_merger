cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid_merger)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  dynamic_reconfigure
  hector_map_tools
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  )

#Find other libaries
find_package(OpenCV REQUIRED)

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES OccupancyGridMerger
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs std_msgs image_transport 
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(OccupancyGridMerger
  src/OccupancyGridMerger.cpp 
  src/Merger2d.cpp
  src/Evolution.cpp
  src/transformations.cpp
  src/Generation.cpp
  src/Population.cpp
  src/Individual.cpp
  src/verification.cpp)

## Add configure headers for dynamic reconfigure
add_dependencies(OccupancyGridMerger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(OccupancyGridMerger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

