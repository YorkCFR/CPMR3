This is a very thin wrapper for the Kinova gen 3 lite arm. This only supports joint angle mode and opening and closing the gripper. But this is enough for some simple tasks. There is a node that provides services for the arm, and a simple testing harness. 

The server can be run as

ros2 run kinova_gen3 kinova_gen3 --ros-args -p simulate:=False

To have the arm not run 'simulated' (which is the default). 

It would be straightforward to wrap all of the other arm modes, but this is likely to be moot once Kinova offers a ROS2 library for the arm.
