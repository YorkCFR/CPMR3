This repository contains code associated with Chapter 6 of CPMR 3rd Editon by G. Dudek and M. Jenkin. 

This chapter provides two simple environment simulations for end-to-end AI-based control of a line following robot and a road following robot. To make a test environment follow the instrucions in the gazebo directory to make two test worlds available for gazebo. Construct a world as described in the text.

* drive_by_road.launch.py - This provides a tool to collect road view imagery for later construction of a NN to drive the robot based on the camera view.
* drive_by_line.launch.py - This provides a tool to collect line view imagery for later construciton of a NN to drive the robot based on the view of a line.
* auto_drive_by_road.launch.py - Drive your robot using the NN constructed and the simulated camera view world.
* auto_drive_by_line.launch.py - Drive your robot using the NN constructed usign the simulated line world.
