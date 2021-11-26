# Copyright 2021 the Autoware Foundation
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


def generate_launch_description():
    """Generate launch description with a single component."""
    container_front = ComposableNodeContainer(
        name="point_type_adapter_container_front",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="point_type_adapter",
                plugin="autoware::tools::point_type_adapter::PointTypeAdapterNode",
                name="lgsvl_cloud_converter_node_front",
                remappings=[
                    ("/points_raw", "/lidar_front/points_raw"),
                    ("/points_xyzi", "/lidar_front/points_xyzi"),
                ],
            )
        ],
        output="screen",
    )
    container_rear = ComposableNodeContainer(
        name="point_type_adapter_container_rear",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="point_type_adapter",
                plugin="autoware::tools::point_type_adapter::PointTypeAdapterNode",
                name="lgsvl_cloud_converter_node_rear",
                remappings=[
                    ("/points_raw", "/lidar_rear/points_raw"),
                    ("/points_xyzi", "/lidar_rear/points_xyzi"),
                ],
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container_front, container_rear])
