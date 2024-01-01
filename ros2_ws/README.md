The ROS2 tools here depend on

* ros2-humble-desktop - basic humble install
* python3-colcon-common-extensions - colcon
* python3-opencv - opencv
* ros-humble-navigation2 - nav2 library
* ros-humble-nav2-bringup - nav2 bringup tools
* ros-humble-xacro - support for xacro

Note that (at present) building this package in ros2 humble with the default tools will produce a deprecated warning message related to revisions in setuptools and the ros2 humble package structure. You can avoid seeing these warning by adding

PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS

to your environment.

If you accessed this from CUP, you will have received this as a large tar or zip archive. You should also be able to find this repository on GitHub. The GitHub version may be updated from the version provided by CUP.

