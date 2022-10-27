cmake_minimum_required(VERSION 3.0.2)
project(gps_to_odom)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  geodesy
  geographic_msgs
)

## catkin specific configuration ##
catkin_package(
 INCLUDE_DIRS
 include
#  LIBRARIES 
#  gps_to_odom
 CATKIN_DEPENDS 
 geometry_msgs 
 nav_msgs 
 roscpp 
 roslib 
 sensor_msgs
 geodesy
 geographic_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/gps_to_odom.cpp
# )

## Specify libraries to link a library or executable target against
add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom
  ${catkin_LIBRARIES}
)