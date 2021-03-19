#!/usr/bin/env python3

"""Launch a Gazebo server with an empty world and initialize ROS with command line arguments."""
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
import os
from pathlib import Path
os.chdir(os.path.join(str(Path.home()), "turtlebot3_ws", "src", "ros2_explorer","explorer_gazebo"))
def generate_launch_description():
    # TODO(anyone): Forward command line arguments once that's supported, see
    # https://github.com/ros2/launch/issues/107
    gzserver = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
         'worlds/map1.world.xml'],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                arguments=['-entity', 'turtlebot3', '-file', 'models/turtlebot3_burger/model.sdf'],
                output='screen')

    return LaunchDescription([
        gzserver, spawn_entity
    ])
