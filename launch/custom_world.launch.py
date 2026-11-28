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
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _create_actions(context):
    pkg_share = get_package_share_directory('frontier_exploration_ros2')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)

    world_value = LaunchConfiguration('world').perform(context).strip()
    if os.path.isabs(world_value):
        resolved_world = os.path.abspath(world_value)
    else:
        resolved_world = os.path.join(pkg_share, 'worlds', world_value)

    source_world = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'worlds',
        world_value,
    )
    if not os.path.isfile(resolved_world) and os.path.isfile(source_world):
        resolved_world = os.path.abspath(source_world)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': resolved_world}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )
    return [gzserver_cmd, gzclient_cmd, robot_state_publisher_cmd, spawn_turtlebot_cmd]


def generate_launch_description():
    pkg_share = get_package_share_directory('frontier_exploration_ros2')
    default_world = os.path.join(pkg_share, 'worlds', 'mixed_rooms.world')

    source_world = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'worlds',
        'mixed_rooms.world',
    )
    if not os.path.isfile(default_world) and os.path.isfile(source_world):
        default_world = source_world

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        OpaqueFunction(function=_create_actions),
    ])
