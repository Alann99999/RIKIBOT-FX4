cmake_minimum_required(VERSION 2.8.3)
project(rikibot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf nav_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

