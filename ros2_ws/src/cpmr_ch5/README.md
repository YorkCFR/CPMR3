This repository contains code associated with Chapter 5 of CPMR 3rd Editon by G. Dudek and M. Jenkin. Specific ros2 launch files provided include

* aruco-robot.launch.py - This provides a simulated aruco target and robot
* camera-robot.launch.py - A basic simulated differential drive robot with a camera
* canny-edges-camera-robot.launch.py - A simulated robot with a camera running Canny edge detection
* good-features-camera-robot.launch.py - A simulated robot with a camera running good features to track
* harris-corners-camera-robot.launch.py - A simulated robot with a camera running Harris corner detector
* view-camera-robot.launch.py - A simple node that displays the value of /mycamera/image_raw 

In addition to these launch files, the following ROS run file is provided

* opencv_camera - This connects to /dev/Video0 using opencv and publishes Images on /mycamera/image_raw

To use the Aruco targets provided (i) follow the instructions in the gazebo_models directory to 
add the models to your local gazebo library. Once you do this, run the aruco-robot.launch.py
There will be no target available. However, under the insert tab in Gazebo you will see entries for
a number of Aruco tag objects. Insert one and drag it to the location you want in the world, to 
see the tag being recognized by ROS.

