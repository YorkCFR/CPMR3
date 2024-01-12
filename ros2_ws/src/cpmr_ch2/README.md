This package supports a simple block robot
* gazebo.launch.py - This launches a simple block robot at the origin in gazebo. Note: this is a copy of the launch file provided in cpmr_apb
* build_map.launch.py - This populates the world from a map defined as a json file of cylinders. Note: this only loads json files copied/built into the installation directory.

Different maps can be loaded through the map parameter
* destroy_map.launch.py - Undoes what build_map.launch.py does

This package also supports moving the robot around. The controller has a goal location (goal_x, goal_y, goal_t). When you launch the controller you can utilize the default goal (0,0,0) using

ros2 run cpmr_ch2 drive_robot

or pass a desired initial goal using

ros2 run cpmr_ch2 drive_to_goal --ros-args -p goal_x=1.0 goal_y=2.0 goal_t=1.5

Note that goals must be floating point values.

Once running you can change a goal by setting the parameter.For example, to set the x goal to 1.0

ros2 param set /move_robot_to_goal goal_x 1.0
