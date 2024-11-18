import os
import json
import sys
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    nchairs = 5
    for arg in sys.argv: # there must be a better way...
        if arg.startswith('nchairs:='):
           print(arg.split('chairs:=', 1)[1])
           nchairs = int(arg.split('chairs:=', 1)[1])
        elif ':=' in arg:
           print(f"Unknown argument in {arg}")
           sys.exit(0)
    print(f"Launching {nchairs}")

    nodelist = []

    # fire up gazebo
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    q = IncludeLaunchDescription(gazebo)
    nodelist.append(q)

    # process each chair
    urdf = os.path.join(get_package_share_directory('cpmr_ch11'), 'wheelchair.urdf.xacro')

    for chair in range(nchairs):
        x = 5 * math.cos(chair * math.pi * 2 / nchairs)
        y = 5 * math.sin(chair * math.pi * 2 / nchairs)
        name = f'chair_{chair}'
        print(f"Processing {chair}")
        robot_desc = xacro.process_file(urdf, mappings={'name' : name, 'show_video': 'True', 'camera_tilt': '0.25'}).toxml()
        nodelist.append(
            Node(
                namespace = name,
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='screen',
                arguments=["0", "0", "0", "0", "0", "0", "1", "map", f"{name}/odom"])
            )
        nodelist.append(
            Node(
                namespace = name,
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False, 'robot_description': robot_desc, 'frame_prefix': name + "/"}],
                arguments=[urdf])
            )
        nodelist.append(
            Node(
                namespace = name,
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/" + name + "/robot_description",  "-entity",  name, "-x", str(x), '-y', str(y), '-Y', '0'])
            )
    return LaunchDescription(nodelist)
