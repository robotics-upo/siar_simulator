# SIAR_SIMULATOR

Simulator for SIAR ROBOT in a sewer environment.

### Description

This simulator represents the SIAR robotics platform which is made to navigate through the sewer environment. In the simulator, the model robot can change its width and move with differential configuration, obtain information from cameras and Velodyne Laser.    

The simulator has two main packages: `siar_gazebo` and `siar_plugins`. The first contains the necessary to use the SIAR model in four different versions (v.1: 7 cameras, v.2: 3 cameras, v.3: without cameras, v.4: 6 cameras + velodyne) and also the sewer environment, modifiable from launch by adding or removing sewer sections. The second contains the plugins that allow us to move wheels,change width, and navigate using only cameras `plugin_siar_wheel_piston.cpp` or cameras+velodyne `plugin_siar_velodyne.cpp`.

This simulator was tested and developed in Gazebo 7, ROS Kinetic and Ubuntu 16.04 LTS.

### Dependencies 

This package has dependency on "ros-kinetic-velodyne-laserscan" and "ros-kinetic-joy". Can be resolved using:

```
sudo apt-get install ros-kinetic-velodyne-laserscan
sudo apt-get install ros-kinetic-joy
```

To "teleoperate" the simulator it is necessary to incorporate the plugin `siar_teleop_joy.cpp`. This can be found in `siar_driver` package from the repository [https://github.com/robotics-upo/siar_packages.git].

To use the "semi autonomous and autonomous navigation mode" in the simulator is necessary to complement with `siar_navegation` package from the repository [https://github.com/robotics-upo/siar_navigation.git].

### Usage

The simulator has three different modes to be used: 
* Teleoperate
* Semi-autonomous
* Autonomous

#### Teleoperation mode

With your workspace ready, to start with the teleoperation mode, you should press the start button to have fun. 

#### Semi-autonomous and Autonomous mode

To use "semi-autonomous" or "autonomous" mode is necessary to follow the next steps:

1. Execute: 
```
roslaunch siar_gazebo siar_simulator_(your_favorite_scenario).launch
``` 
*IMPORTANT*: the launch in siar_simulator starts in pause to avoid conflict in the spawn of the models Gazebo. The `siar_costmap` module will generate some errors that will no longer thrown just by pushing play in the Gazebo simulator.
    
2. Execute: 
```
roslaunch siar_planner planner_action_server_simulation.launch
``` 
To use different planners change the parameter `planner_type`. The semi-autonomous mode corresponds to `operation_mode = 1`, you can change with button "Y" of Xbox Joystick or publishing the value 1 in the topic `/operation_mode` .

3. To use "autonomous" mode, you should publish in topic `/operation_mode` the value 100.

4. Launch RVIZ and enjoy!! you just should give a goal in RVIZ inside the sewer environment.
