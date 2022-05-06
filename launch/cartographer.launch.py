# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_collab_mapping = get_package_share_directory('collab_mapping')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  pkg_collab_mapping, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    namespace = LaunchConfiguration('namespace', default=None)
    map_topic = LaunchConfiguration('map_topic', default=[])

    rviz_config_dir = os.path.join(pkg_collab_mapping,
                                   'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value=None,
            description='Namespace used for the robot'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/robot0/scan', '/robot0/filtered/scan'),
                ('/robot1/scan', '/robot1/filtered/scan'),
                ('/robot2/scan', '/robot2/filtered/scan'),
                ('/robot3/scan', '/robot3/filtered/scan'),
                ('/robot4/scan', '/robot4/filtered/scan'),
                ('/robot5/scan', '/robot5/filtered/scan'),
                ('/robot6/scan', '/robot6/filtered/scan'),
                ('/robot7/scan', '/robot7/filtered/scan'),
                ],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
        
        # DeclareLaunchArgument(
        #     'map_topic',
        #     default_value=None,
        #     description='/map topic name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec, 'namespace': namespace}.items(),
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     namespace=namespace,
        #     remappings=[('/map', map_topic)],
        #     # remappings=[
        #     #     ('/map', f'/{namespace.perform()}/map'),
        #     #     ('/map_updates', f'/{namespace.perform()}/map_updates'),
        #     #     ('/scan', f'/{namespace.perform()}/scan'),
        #     #     ('/scan_matched_points2', f'/{namespace.perform()}/scan_matched_points2'),
        #     #     ('/clicked_point', f'/{namespace.perform()}/clicked_point'),
        #     #     ('/move_base_simple/goal', f'/{namespace.perform()}/move_base_simple/goal')
        #     # ],
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
