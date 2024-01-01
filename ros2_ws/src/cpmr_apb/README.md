This package provides support for the tutorial which can be found at the root of this repository.
* cpmr_apb gazebo.launch.py  - A launch file that puts a very simple block robot in an empty world
* populate_world  - A ROS application that populates the world with coke cans. Note: This assumes that gazeo has/can download the coke can model. This may result in some delay the first itme it is run.
* depopulate_word - A  ROS application that depopulates the world full of coke cans.

ros2 launch cpm4_apb gazebo.launch.py 

Will launch the block robot in an empty gazebo world. While this world is running, populate_world will populate the world with cans, assuming you have already loaded the model into your local gazebo repository. depopulate_world will remove them.

You can control the block robot using the teleop_twist_keyboard application found in the teleop_twist_keyboard package

ros2 run teleop_twist_keyboard teleop_twist_keyboard
