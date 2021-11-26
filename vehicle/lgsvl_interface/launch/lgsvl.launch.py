# Copyright 2020 the Autoware Foundation
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launch necessary dependencies for working with LGSVL simulator and ROS 2/Autoware.Auto.

    The LGSVL interface, which translates inputs and outputs to and from ROS standard coordinate
    systems, and the ros2 web bridge, which allows LGSVL to pick up ROS 2 topics.
    """
    # --------------------------------- Params -------------------------------

    # In combination 'raw', 'basic' and 'high_level' control
    # in what mode of control comands to operate in,
    # only one of them can be active at a time with a value
    control_command_param = DeclareLaunchArgument(
        "control_command",
        default_value="ackermann",  # use "raw", "basic" or "high_level"
        description="command control mode",
    )

    # Default lgsvl_interface params
    lgsvl_interface_param = DeclareLaunchArgument(
        "lgsvl_interface_param",
        default_value=[get_share_file("lgsvl_interface", "param/lgsvl.param.yaml")],
        description="Path to config file for lgsvl interface",
    )

    # -------------------------------- Nodes-----------------------------------

    # LGSVL interface
    lgsvl_interface = Node(
        package="lgsvl_interface",
        executable="lgsvl_interface_exe",
        namespace="vehicle",
        output="screen",
        parameters=[
            LaunchConfiguration("lgsvl_interface_param"),
            # overwrite parameters from yaml here
            {"control_command": LaunchConfiguration("control_command")},
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom"),
            ("ackermann_vehicle_command", "/control/command/control_cmd"),
            ("twist", "/vehicle/status/velocity_status"),
            ("steer_report", "/vehicle/status/steering_status"),
            ("gear_report", "/vehicle/status/gear_status"),
            ("control_mode_report", "/vehicle/status/control_mode"),
            ("odom", "/localization/kinematic_state"),
        ],
    )

    ld = LaunchDescription([control_command_param, lgsvl_interface_param, lgsvl_interface])
    return ld
