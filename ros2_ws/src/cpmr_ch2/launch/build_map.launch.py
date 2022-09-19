import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value = 'default.json', description = 'Map name'),
        Node(
            package = 'cpmr_ch2',
            executable = 'build_map',
            name = 'build_map',
            parameters = [
                {'map' : LaunchConfiguration('map')},
            ],
        ),
    ])

