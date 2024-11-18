Chapter 11 provides the following launch files

* chairs.launch.py - launch a collection of wheelchairs
* chairs_controller.launch.py - launch a simple set of controllers for the chairs. chair_0 is the leader, the other chairs follow the leader

This provides an example of how multiple robots can be deployed in a common space with different name spaces for each robot. Both launch files take nchairs:=n where n is a positive integer for the number of chairs to launch/control

Note: the individual chair controllers must be started using 

ros2 service call /chair_0/startup std_srvs/srv/SetBool '{data: True}'

for each of the chairs. chair_0 is the leader


