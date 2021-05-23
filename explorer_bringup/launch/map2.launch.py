#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_name = LaunchConfiguration('map_name', default='map2')
    world_file_name = 'map2.world.xml'
    world = os.path.join(get_package_share_directory('explorer_gazebo'),
                         'worlds', world_file_name)
    gazebo_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    cartographer_launch_file_dir = os.path.join(get_package_share_directory('explorer_cartographer'), 'launch')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('explorer_navigation2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_launch_file_dir, '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='explorer_wanderer',
            executable='wanderer_server',
            name='wanderer_server',
            output='screen',
        ),
        Node(
            package='explorer_wanderer',
            executable='discoverer_server',
            name='discoverer_server',
            output='screen',
        ),
        Node(
            package='explorer_map_utils',
            executable='watchtower',
            name='watchtower',
            output='screen',
            parameters=[{'map_name': map_name}],
        ),


    ])