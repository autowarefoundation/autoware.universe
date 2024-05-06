#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
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

# Author: Brighten Lee
#         Hongrui Zheng

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_locks = os.path.join(get_package_share_directory('ackermann_mux'),
                                        'config', 'ackermann_mux_locks.yaml')
    default_config_topics = os.path.join(get_package_share_directory('ackermann_mux'),
                                         'config', 'ackermann_mux_topics.yaml')
    default_config_joystick = os.path.join(get_package_share_directory('ackermann_mux'),
                                           'config', 'joystick.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='ackermann_mux/cmd_vel',
            description='cmd vel output topic'),
        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[
                LaunchConfiguration('config_locks'),
                LaunchConfiguration('config_topics'),
                LaunchConfiguration('config_joy')]
        ),
            ])
