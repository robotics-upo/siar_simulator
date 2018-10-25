# siar_simulator

This simulator represent the SIAR ROBOT and is controlled by joystick in a environment of sewer.

Basicly the simulator is a workspace with two package, siar_gazebo and siar_plugins. The first contain the necesary to create the model of the robot and the environment, and the second is to control the movement and widht of the robot through joystick.

This simulator was tested in developed in Gazebo 7, ROS Kinetic and Ubuntu 16.04 LTS. For that reason, for a proper use it is recommended:
  - Install the full version ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - Install Gazebo 7 to Ubuntu from gazebosim.com (http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
  - Install the joy package from ROS. (http://wiki.ros.org/joy)

Once it has been installed the programs you can to incorporate the workspace "siar_simulator" and after (catkin_make) it is possible to launch (roslaunch siar_gazebo siar_model.launch) the simulator. You should check the input that is configurated your joystick (modifiable in siar_model.launch)
