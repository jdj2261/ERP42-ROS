################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(erp42_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  erp42_msgs
)

link_directories(
    /usr/lib
)

link_libraries(
    pcanbasic
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf erp42_msgs
)

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(erp42_driver_node
  src/erp42_driver.cpp
  src/erp42_driver_node.cpp
  src/ERP42Interface/erp42_interface.cpp
)
add_dependencies(erp42_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(erp42_driver_node ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################


################################################################################
# Test
################################################################################
