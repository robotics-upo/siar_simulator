## STEPS TO LAUNCH SIAR_SIMULATOR

#SUMMARY

SIAR_SIMULATOR package has three different siar model:
    1.- with 7 cameras orbec (siar.sdf)
    2.- with 7 cameras orbec + velodyne (siar_v.sdf)
    3.- with 7 cameras orbec + velodyne + thermal camera + gps (siar_mb.sdf)
Also there different models and world that allow to create different environment to test the robot.

#SELECT SIAR_MODEL
To selec the model, in the launch modify in the spawn_model node the argument "-model", then write yhe model that you want to use(siar.sdf, siar_v.sdf, siar_mb.sdf).

SELECT WORLD
To selec a world, change the argument "world" in the launch". The worlds ables are "siar_empty.world" and "siar_mbzirc.wordl".

#FIRST LAUNCH
To launch a siar model in the empty wordl , use the "roslaunch siar_gazebo siar_model_complete.launch"

