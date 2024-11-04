Code to demonstrate simple software architectures for autonomous robots

- subsumption_robot.launch.py shows a simple subsumption architecture. This robot will wander randomly, avoiding objects, and will be attracted by red objects. (The detection of red is neither robust nor particularly effective.) There is a sample red object in the package that can be used to provide a suitable red object.

Once launched, the robot can be started up using the service call

ros2 service call /startup std_srvs/srv/SetBool '{data: True}'

- fsm.launch.py shows a simple finite sate machine architecture. Once started, the robot will execute a path which is defined in the blackboard.

To start the robot, use the service call

ros2 service call /startup std_srvs/srv/SetBool '{data: True}'


