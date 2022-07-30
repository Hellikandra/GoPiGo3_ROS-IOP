#!/bin/bash
sudo apt install deault-jdk -y

# depending on what the catkin was set, use catkin or catkin_tools.
sudo apt install python-wstool -y

cd catkin_ws # in /home/<user>/catkin_ws

# Merge the iop.rosinstall file and fetch the code
# If you do not already have an .rosinstall go into you ROS workspace and call
# cd catkin_ws/; wstool init src/iop

wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_core/master/iop.rosinstall
wstool update -t src/iop

# idk why but we need to install this first
sudo apt update
sudo apt install ros-melodic-geographic-msgs

# Install dependencies
rosdep install --from-paths src --ignore-src

# In the ROS workspace :
catkin_make -j1 #-j1 for Raspberry-Pi 3 environment.
# or catkin build

# Case one one Ubuntu VBOX
# watch out : you need to do
wstool init src # to re set a .rosinstall
wstool merge -t src http://githubusercontent.com/fkie/multimaster_fkie/master/fkie_multimaster.rosinstall
wstool update -t src

# Case two on Raspberry Pi 3
git clone https://github.com/fkie/multimaster_fkie.git multimaster
rodep update

# add this before 
# apt install python-dev --fix-missing
pip2 install wheel
pip install cython # pip2 does not reach the cython package
# pip2 install <package>
# pip2 freeze # currently package installed
# pip2 search <query>
# pip2 uninstall <package>

rosdep install -i --as-root pip:false --reinstall --from-paths multimaster

# build all package
# do it in ROS Workspace (~/catkin_ws)
catkin build fkie_multimaster

catkin_make fkie_multimaster


## avant de faire un rosrun, ne pas oublier de faire 
source devel/setup.bash 



# before example :
sudo apt install docker docker.io
sudo apt install ros-melodic-turtlesim ros-kinetic-rqt-gui


wstool merge -t src/iop https://raw.githubusercontent.com/fkie/iop_examples/master/iop_examples.rosinstall
wstool update -t src/iop
sudo apt install python3-pip or python2-pip # required by iop_examples
catkin_make # or catkin build
source devel/setup.bash 
https://github.com/fkie/iop_examples/tree/master/fkie_iop_cfg_sim_turtle

## install docker to use it !!
sudo apt install docker docker.io
cd /home/<username>/catkin_ws/src/iop/iop_examples # you normally see a Dockerfile

sudo apt-get install -y apt-utils
docker build -t fkie:iop .
