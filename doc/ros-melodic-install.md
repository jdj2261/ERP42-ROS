# ROS melodic 설치

작성자 : djjin

ubuntu 18.04 melodic 버전 ROS 설치 매뉴얼

http://wiki.ros.org/melodic/Installation/Ubuntu

[TOC]

## 1. Installation

~~~
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-melodic-desktop-full
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo rosdep init
$ rosdep update
~~~



- Install dependencies using rosdep

~~~
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
~~~

