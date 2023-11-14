# Copyright 2020 Tier IV, Inc. All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationNotEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    # https://github.com/ros2/launch_ros/issues/156
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    crop_box_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_measurement_range",
        remappings=[
            ("input", LaunchConfiguration("input/pointcloud")),
            ("output", "measurement_range/pointcloud_ex"),
        ],
        parameters=[
            # load_composable_node_param("crop_box_filter_measurement_range_param_path"),
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "min_x": -10.0,
                "max_x": 10.0,
                "min_y": 0.0,
                "max_y": 7.5,
                "min_z": -5.0,
                "max_z": 5.0,
                "negative": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ring_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PassThroughFilterUInt16Component",
        name="ring_filter",
        remappings=[
            ("input", "measurement_range/pointcloud_ex"),
            ("output", "ring_filter/pointcloud_ex"),
        ],
        parameters=[
            # load_composable_node_param("crop_box_filter_measurement_range_param_path"),
            {
                "input_frame": "base_link",
                "output_frame": "base_link",
                "filter_field_name": "ring",
                "filter_limit_min": 5,
                "filter_limit_max": 45,
                "filter_limit_negative": False,
                "keep_organized": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    composable_nodes = [
        crop_box_component,
        ring_filter_component,
    ]

    target_container = (
        "/sensing/lidar/top/pointcloud_preprocessor/pointcloud_container"
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("pointcloud_container_name")
    )

    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals(target_container, ""),
        composable_node_descriptions=composable_nodes,
        target_container=target_container,
    )

    return [load_composable_nodes]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        arg = DeclareLaunchArgument(name, default_value=default_value, description=description)
        launch_arguments.append(arg)

    # add_launch_arg(
    #     "crop_box_filter_measurement_range_param_path",
    #     [
    #         LaunchConfiguration("crop_box_filter_measurement_range_param_path"),
    #     ],
    #     "path to the parameter file of crop_box_filter_measurement_range",
    # )

    add_launch_arg("use_intra_process", "true", "use ROS 2 component container communication")
    add_launch_arg("use_pointcloud_container", "True", "use pointcloud container")
    add_launch_arg(
        "pointcloud_container_name",
        "/pointcloud_container",
        "container name",
    )

    add_launch_arg(
        "output/pointcloud",
        "downsample/pointcloud",
        "final output topic name",
    )

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
