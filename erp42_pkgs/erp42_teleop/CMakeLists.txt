cmake_minimum_required(VERSION 3.0.2)
project(erp42_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    ackermann_msgs
    erp42_msgs
  DEPENDS
)

