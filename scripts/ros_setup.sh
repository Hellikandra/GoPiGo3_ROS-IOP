#!/bin/bash
# http://wiki.ros.org/melodic/Installation/Ubuntu
# https://roboticsbackend.com/install-ros-on-raspberry-pi-3/
# ending by 
# $ mkdir -p ~/catkin_ws/src
# $ cd ~/catkin_ws
# $ catkin_make
# https://titanwolf.org/Network/Articles/Article?AID=ffc27712-a7fc-4ccf-84b8-0cc8d235a94f#gsc.tab=0
# 1.2 Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# 1.3 Setup up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# 1.4 Installation
sudo apt update
## Depending on the Ubuntu version, install melodic for 18.04 & noetic for 20.04
sudo apt install ros-melodic-desktop-full # include ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception
# 1.5 Environment setup
# adding line in ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# changing the environment in current shell : only to finish the current install
source /opt/ros/melodic/setup.bash
source /opt/ros/noetic/setup.bash
# 1.6 Dependencies for building packages
# Melodic version :
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
# Noetic version :
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep

## Both version use the same
sudo rosdep init
rosdep update

## Install catkin versus using catkin_make
# sudo apt install python-tools-catkin
# for Noetic : sudo apt install python3-catkin-tools python3-osrf-pycommon

## create the workspace of ROS
# /home/<username>/catkin_ws ..
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
#catkin build 
catkin_make
source devel/setup.bash
# now do more stuff for ROS or you can now install IOP stack from fkie
# if everything was correct :
echo $ROS_PACKAGE_PATH
# must sho ROS Workspace and ROS install share.
