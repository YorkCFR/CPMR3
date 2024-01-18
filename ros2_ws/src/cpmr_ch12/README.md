This code deals with interfacing a USB joystick to ROS2. As every joystick is different you will have to generate a configuration file for your own specific joystick.

The set of parameters that define your joystck should go in a yaml file. Here, a sample yaml file can be found in config/joystick.config.yaml

The launch file camera-robot.launch.py launches the differential drive robot equipped with a camera from Chapter 5. It responds to motion commands on /cmd_vel . In previous chapters /cmd_vel was provided by the teleop_twist_keyboard package. Here we will use the joystick todo this

With the joystick plugged in, you can confirm that your joystick is recognized by ROS using

ros2 run joy joy enumerate_devices

This will enumerate the joysticks that are known. You should see your joystick, and it will most likely be assigned as joystick number zero.

Now run the ros node joy_node in the joy package

ros2 run joy joy_node

This will respond to joystick button and joystick presses. It will be helpful when configuring your joystick to have this running in a separate window.

Now to map joystick commands to /cmd_vel messages. This is accomplished using  the teleop_node in the teleop_twist_joy package

ros2 run teleop_twist_joy teleop_node --ros-args --params-file src/cpmr_ch12/config/joystick.config.yaml

where the params-file parameter is the path to the joystick configuration file. Configure your joystick to drive this simulated robot around. Full details on the available parameters can be found in the online documentation for ROS2.
