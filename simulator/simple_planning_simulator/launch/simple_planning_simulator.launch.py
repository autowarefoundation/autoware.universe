# Copyright 2021 The Autoware Foundation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ament_index_python
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

import os


def generate_launch_description():

    default_vehicle_characteristics_param = os.path.join(
        get_package_share_directory('simple_planning_simulator'),
        'param/vehicle_characteristics.param.param.yaml')

    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=default_vehicle_characteristics_param,
        description='Path to config file for vehicle characteristics'
    )

    simple_planning_simulator_node = launch_ros.actions.Node(
        package='simple_planning_simulator',
        executable='simple_planning_simulator_exe',
        name='simple_planning_simulator',
        namespace='simulation',
        output='screen',
        parameters=[
            "{}/param/simple_planning_simulator_default.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "simple_planning_simulator"
                )
            ),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
        remappings=[
            ('input/vehicle_control_command', '/vehicle/vehicle_command'),
            ('input/ackermann_control_command', '/vehicle/ackermann_vehicle_command'),
            ('input/vehicle_state_command', '/vehicle/state_command'),
            ('output/kinematic_state', '/vehicle/vehicle_kinematic_state'),
            ('output/vehicle_state_report', '/vehicle/state_report'),
            ('/initialpose', '/localization/initialpose'),
        ]
    )

    map_to_odom_tf_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_tf_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'])

    ld = launch.LaunchDescription([
        vehicle_characteristics_param,
        simple_planning_simulator_node,
        map_to_odom_tf_publisher
    ])
    return ld
