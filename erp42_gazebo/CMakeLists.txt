cmake_minimum_required(VERSION 3.0.2)
project(erp42_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()
catkin_install_python(PROGRAMS scripts/ackermann_controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#############
## Install ##
#############
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
