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
from launch_ros.descriptions import ComposableNode


def create_component(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="default_ad_api/helpers",
        name=node_name,
        package="default_ad_api_helpers",
        plugin="default_ad_api_helpers::" + class_name,
        **kwargs
    )


def generate_launch_description():
    remap_routing = [
        ("~/input/goal", "/planning/mission_planning/goal"),
        ("~/input/waypoint", "/planning/mission_planning/checkpoint"),
    ]
    components = [
        create_component("routing", "RoutingAdaptor", remappings=remap_routing),
    ]
    container = ComposableNodeContainer(
        namespace="default_ad_api",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
    )
    return launch.LaunchDescription([container])
