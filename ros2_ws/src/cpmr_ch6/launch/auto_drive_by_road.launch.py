import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('cpmr_ch6'), 'scout-camera.urdf.xacro')
    robot_desc = xacro.process_file(urdf, mappings={'name' : 'sonar_robot'}).toxml()

    return LaunchDescription([
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                           'launch', 'gazebo.launch.py'),
             )
        ),

        Node(
             package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'use_sim_time': 'false', 'robot_description': robot_desc}],
             arguments=[urdf]),
        Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             name='urdf_spawner',
             output='screen',
             arguments=["-topic", "/robot_description",  "-entity",  "camera_robot"]),
        Node(
             package='cpmr_ch6',
             executable='auto_drive_by_road',
             name='auto_drive_by_road',
             output='screen'),
    ])

