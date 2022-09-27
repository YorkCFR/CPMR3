import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    return LaunchDescription([
        Node(
             package='cpmr_ch4',
             executable='collect_lidar',
             name='collect_lidar',
             output='screen'),
    ])

