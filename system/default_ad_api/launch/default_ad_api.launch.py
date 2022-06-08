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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def _create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="default_ad_api/node",
        name=node_name,
        package="default_ad_api",
        plugin="default_ad_api::" + class_name,
        **kwargs
    )


def generate_launch_description():
    components = [
        _create_api_node("interface", "InterfaceNode"),
        _create_api_node(
            "motion_factor_aggregator_node",
            "MotionFactorAggregator",
            remappings=[
                (
                    "input/scene_module/motion_factor",
                    "/planning/scenario_planning/status/scene_modules/motion_factors",
                ),
                (
                    "input/obstacle_stop/motion_factor",
                    "/planning/scenario_planning/status/obstacle_stop/motion_factors",
                ),
                (
                    "input/surround_obstacle/motion_factor",
                    "/planning/scenario_planning/status/surround_obstacle_checker/motion_factors",
                ),
                ("input/autoware_trajectory", "/planning/scenario_planning/trajectory"),
                ("~/output/motion_factors", "/api/get/motion_factors"),
            ],
            parameters=[
                {"status_pub_hz": 5.0},
                {"timeout": 0.5},
            ],
        ),
    ]
    container = ComposableNodeContainer(
        namespace="default_ad_api",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
    )
    web_server = Node(
        package="default_ad_api",
        name="web_server",
        executable="web_server.py",
    )
    return launch.LaunchDescription([container, web_server])
