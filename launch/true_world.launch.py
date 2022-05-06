#!/usr/bin/env python3

from email.policy import default
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


TURTLEBOT3_MODEL = "waffle"


def read_init_poses(filename):
    init_poses = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            init_poses.append(line.split(','))
    
    print(init_poses)
    
    return init_poses

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_collab_mapping = get_package_share_directory('collab_mapping')

    world_file_name = 'true.world'
    world = os.path.join(pkg_collab_mapping, 'worlds', world_file_name)
    init_poses = read_init_poses(os.path.join(pkg_collab_mapping, 'waypoints', 'init_poses.csv'))

    descriptions = []
    descriptions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    ))

    descriptions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    ))

    for i in range(len(init_poses)):
        descriptions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_collab_mapping, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': f'robot{i}'}.items(),
        ))

        descriptions.append(Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            namespace=f'robot{i}',
            arguments=['-entity', f'{TURTLEBOT3_MODEL}{i}', 
                    '-file', os.path.join(pkg_collab_mapping, 'models/model.sdf'),
                    '-x', init_poses[i][0],
                    '-y', init_poses[i][1],
                    '-z', '0.0',
                    '-Y', '0.0',
                    '-robot_namespace', f'robot{i}'],
                    output='screen'
        ))


        descriptions.append(Node(
            package='collab_mapping',
            executable='turtlebot_control',
            namespace=f'robot{i}',
            arguments=[
                '-file', os.path.join(pkg_collab_mapping, f'waypoints/robot{i}.csv'),
                '-namespace', f'robot{i}']
        ))

        descriptions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_collab_mapping, 'launch', 'cartographer.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'namespace': f'robot{i}'}.items(),
        ))

    
        descriptions.append(Node(
            package='collab_mapping',
            executable='laser_filter',
            namespace=f'robot{i}',
            arguments=['-namespace', f'robot{i}']
        ))


    return LaunchDescription(descriptions)
