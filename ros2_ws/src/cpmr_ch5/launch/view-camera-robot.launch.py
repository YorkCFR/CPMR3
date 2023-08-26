import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('cpmr_ch5'), 'scout-camera.urdf.xacro')
    robot_desc = xacro.process_file(urdf, mappings={'name' : 'camera_robot'}).toxml()

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
             parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
             arguments=[urdf]),
        Node(
             package='gazebo_ros',
             executable='spawn_entity.py',
             name='urdf_spawner',
             output='screen',
             arguments=["-topic", "/robot_description",  "-entity",  "camera_robot"]),
        Node(
             package='cpmr_ch5',
             executable='view_camera',
             name='view_camera',
             output='screen'),
    ])

