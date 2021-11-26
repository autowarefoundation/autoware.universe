# Copyright 2020-2021 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launch a minimal joystick + LGSVL demo.

    The joystick_vehicle_interface_nodes and the lgsvl_interface
    are modified via parameter remapping to use VehicleControlCommand as an output.
    The vehicle can be controlled by manipulating the left joystick of the gamepad.
    """
    # --------------------------------- Params -------------------------------

    # In combination 'raw', 'basic' and 'high_level' control
    # in what mode of control comands to operate in,
    # only one of them can be active at a time with a value
    control_command_param = DeclareLaunchArgument(
        "control_command",
        default_value="raw",  # use "raw", "basic" or "high_level"
        description="command control mode topic name",
    )

    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        "joy_translator_param",
        default_value=[
            get_share_file("joystick_vehicle_interface_nodes", "param/logitech_f310_raw.param.yaml")
        ],
        description="Path to config file for joystick translator",
    )

    # Default lgsvl_interface params
    lgsvl_interface_param = DeclareLaunchArgument(
        "lgsvl_interface_param",
        default_value=[get_share_file("lgsvl_interface", "param/lgsvl.param.yaml")],
        description="Path to config file for lgsvl interface",
    )

    # -------------------------------- Nodes-----------------------------------
    # Include Joystick launch
    joystick_launch_file_path = get_share_file(
        "joystick_vehicle_interface_nodes", "launch/joystick_vehicle_interface_node.launch.py"
    )
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments={
            "joy_translator_param": LaunchConfiguration("joy_translator_param"),
            "control_command": LaunchConfiguration("control_command"),
        }.items(),
    )

    # Include LGSVL interface launch
    lgsvl_launch_file_path = get_share_file("lgsvl_interface", "launch/lgsvl.launch.py")
    lgsvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments={
            "lgsvl_interface_param": LaunchConfiguration("lgsvl_interface_param"),
            "control_command": LaunchConfiguration("control_command"),
        }.items(),
    )

    return LaunchDescription(
        [control_command_param, joy_translator_param, lgsvl_interface_param, joystick, lgsvl]
    )
