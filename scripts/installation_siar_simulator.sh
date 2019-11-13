#!/bin/bash

#Installation GAZEBO

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7
sudo apt-get install libgazebo7-dev


#Installing ROS dependencies 

sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control

sudo apt-get install ros-$ROS_DISTRO-velodyne-gazebo-plugins ros-$ROS_DISTRO-twist-mux ros-$ROS_DISTRO-joy libignition-math3-dev

mkdir -p ~/siar_ws/src
cd ~/siar_ws/src
git clone https://github.com/robotics-upo/siar_packages.git 
git clone https://github.com/robotics-upo/functions.git

cd ~/siar_ws
catkin_make --pkg siar_arm
catkin_make --pkg siar_driver
catkin_make