import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('cpmr_apb'), 'blockrobot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': 'false', 'robot_description': robot_desc}],
             arguments=[urdf]),
    ])
