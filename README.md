# SIAR_SIMULATOR

Simulator for SIAR ROBOT in a sewer environment.

### Description

The simulator has two package, "siar_gazebo" and "siar_plugins". The first contain the necesary to use the SIAR model in four different versions (v.1: 7 cameras, v.2: 3 cameras, v.3: without camera, v.4: 6 cameras + velodyne) and also the sewer environment which is modifiable within launch, adding or removing sewer sections. The second contains the plugins that allow  move wheels,change widht, and navigate using only cameras `plugin_siar_wheel_piston.cpp` or cameras+velodyne `plugin_siar_velodyne.cpp`.

This simulator was tested in developed in Gazebo 7, ROS Kinetic and Ubuntu 16.04 LTS.


### Dependencies 

This package has dependency with "ros-kinetic-velodyne-laserscan" and "ros-kinetic-joy". Can be resolved using:

```
sudo apt-get install ros-kinetic-velodyne-laserscan
sudo apt-get install ros-kinetic-joy
```
Also, to teleoperate the simulator is necesarry to incorporate the plugin `siar_teleop_joy.cpp`. This can be find in the repository [https://github.com/robotics-upo/siar_packages.git].

### Usage

The simulator has tree different mode to be used: 
* Teleoperate
* Semi autonomous
* Autonomous

#### Teleoperation mode

To "teleoperate" and to implement the "autonomous navigation" in the simulator, this package should be complement with SIAR_NAVEGATION package from rbotics-upo repository.



Once that you will have ready the workspace, to start with the teleoperation mode, you should press start button to have fun. 

To use "semi autonomous" or "autonomous" mode is necessary to follow the next steps:

1.- Execute "roslaunch siar_gazebo siar simulator_(choose your favorite version).launch". 
IMPORTANT: the launch in siar_simulator start in pause to avoid conflict in the spawn of the models Gazebo. This will generate a ROS_ERROR, because siar_costmap is waiting to recieve the map. To finish with ROS_ERROR just push play in the simulation.
    
2.- Execute "roslaunch siar_planner planner_action_server_simulation.launch" (To use differents planners change the parameter "planner type"). Congratulations !!!, now you are ready to use siar_simulator in "semi autonomous" mode, just be sure that you are in "operation mode" = 1, you can change with botton "Y" of Xbox Joystick.

3.- To use "autonomous" mode, you should publish  in topic "operation mode" the value 100.

4.- Launch RVIZ and enjoy!! you just should give a goal inside the sewer environment.


