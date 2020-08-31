################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(erp42_serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  erp42_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp serial std_msgs erp42_msgs
)

################################################################################
# Build
################################################################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(erp42_serial_node
    src/erp42_serial.cpp
    src/erp42_serial_node.cpp
    src/SerialPort/serial_port.cpp
    )
add_dependencies(erp42_serial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(erp42_serial_node ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################


################################################################################
# Test
################################################################################
