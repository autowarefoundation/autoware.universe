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


from pathlib import Path

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
import yaml


def create_diagnostic_name(row):
    return '"default_ad_api: {}_topic_status"'.format(row["module"])


def create_topic_monitor_name(row):
    diag_name = create_diagnostic_name(row)
    return "topic_state_monitor_{}: {}".format(row["args"]["node_name_suffix"], diag_name)


def create_topic_monitor_node(row):
    tf_mode = "" if "topic_type" in row["args"] else "_tf"
    package = FindPackageShare("topic_state_monitor")
    include = PathJoinSubstitution([package, f"launch/topic_state_monitor{tf_mode}.launch.xml"])
    diag_name = create_diagnostic_name(row)
    arguments = [("diag_name", diag_name)] + [(k, str(v)) for k, v in row["args"].items()]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="default_ad_api/node",
        name=node_name,
        package="default_ad_api",
        plugin="default_ad_api::" + class_name,
        **kwargs,
    )


def launch_setup(context, *args, **kwargs):
    # create topic monitors
    mode = LaunchConfiguration("online_mode").perform(context)
    rows = yaml.safe_load(Path(LaunchConfiguration("config_file").perform(context)).read_text())
    rows = rows if mode else [row for row in rows if not rows["only_online_mode"]]
    topic_monitor_nodes = [create_topic_monitor_node(row) for row in rows]
    topic_monitor_names = [create_topic_monitor_name(row) for row in rows]
    param_operation_mode = [{"topic_monitor_names": topic_monitor_names}]

    # create api components
    components = [
        create_api_node("interface", "InterfaceNode"),
        create_api_node("localization", "LocalizationNode"),
        create_api_node("motion", "MotionNode", parameters=[{"require_accept_start": False}]),
        create_api_node("operation_mode", "OperationModeNode", parameters=param_operation_mode),
        create_api_node("routing", "RoutingNode"),
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
            DeclareLaunchArgument("online_mode"),
            OpaqueFunction(function=launch_setup),
            Node(
                package="default_ad_api",
                name="web_server",
                executable="web_server.py",
            ),
        ]
    )
