# Copyright 2020 Tier IV, Inc.
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

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os


context = LaunchContext()


def find_pack(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(
        Path(FindPackage(package_name).perform(context)), 'share', package_name)


def generate_launch_description():

    simple_planning_simulator_param_file = os.path.join(
        find_pack('simple_planning_simulator'),
        'config/simple_planning_simulator.param.yaml')

    print(simple_planning_simulator_param_file)

    simple_planning_simulator_param = DeclareLaunchArgument(
        'simple_planning_simulator_param_file',
        default_value=simple_planning_simulator_param_file,
        description='Path to config file for simple_planning_simulator'
    )

    simple_planning_simulator = Node(
        package='simple_planning_simulator',
        node_executable='simple_planning_simulator_exe',
        node_name='simple_planning_simulator',
        node_namespace='simulation',
        output='screen',
        parameters=[
            LaunchConfiguration('simple_planning_simulator_param_file'),
        ],
        remappings=[
            ('base_trajectory', '/planning/scenario_planning/trajectory'),
            ('output/current_pose', 'current_pose'),
            ('output/current_twist', '/vehicle/status/twist'),
            ('output/status', '/vehicle/status'),
            ('output/control_mode', '/vehicle/status/control_mode'),
            ('input/vehicle_cmd', '/control/vehicle_cmd'),
            ('input/turn_signal_cmd', '/control/turn_signal_cmd'),
            ('input/initial_twist', '/initialtwist'),
            ('input/initial_pose', '/initialpose'),
            ('input/engage', '/vehicle/engage'),
        ]
    )

    return LaunchDescription([
        simple_planning_simulator_param,
        simple_planning_simulator
    ])
