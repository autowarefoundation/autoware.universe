# Copyright 2022 TIER IV, Inc.
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

import csv

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def create_topic_monitor_name(row, diag_name):
    return "topic_state_monitor_{}: {}".format(row["suffix"], diag_name)


def create_topic_monitor_node(row, diag_name):
    package = FindPackageShare("topic_state_monitor")
    include = PathJoinSubstitution([package, "launch/topic_state_monitor.launch.xml"])
    arguments = [
        ("diag_name", diag_name),
        ("node_name_suffix", row["suffix"]),
        ("topic", row["topic"]),
        ("topic_type", row["topic_type"]),
        ("timeout", row["timeout"]),
        ("warn_rate", row["warn_rate"]),
        ("error_rate", row["error_rate"]),
        ("best_effort", row["best_effort"]),
        ("transient_local", row["transient_local"]),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="default_ad_api/node",
        name=node_name,
        package="default_ad_api",
        plugin="default_ad_api::" + class_name,
        **kwargs
    )


def launch_setup(context, *args, **kwargs):
    # create topic monitors
    with open(LaunchConfiguration("config_file").perform(context)) as fp:
        rows = list(csv.DictReader(fp))
    diag_name = "default_ad_api"
    topic_monitor_nodes = [create_topic_monitor_node(row, diag_name) for row in rows]
    topic_monitor_names = [create_topic_monitor_name(row, diag_name) for row in rows]
    params_operation_mode = [{"topic_monitor_names": topic_monitor_names}]

    # create api components
    components = [
        create_api_node("interface", "InterfaceNode"),
        create_api_node("operation_mode", "OperationModeNode", parameters=params_operation_mode),
    ]
    container = ComposableNodeContainer(
        namespace="default_ad_api",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
    )
    return [container, *topic_monitor_nodes]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            OpaqueFunction(function=launch_setup),
            Node(
                package="default_ad_api",
                name="web_server",
                executable="web_server.py",
            ),
        ]
    )
