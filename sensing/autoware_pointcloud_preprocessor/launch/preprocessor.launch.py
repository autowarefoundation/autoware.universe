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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def load_parameter_dic(package_share_directory, yaml_file_path):
    param_file = os.path.join(get_package_share_directory(package_share_directory), yaml_file_path)

    with open(param_file, "r") as file:
        param_dic = yaml.safe_load(file)["/**"]["ros__parameters"]

    return param_dic


def launch_setup(context, *args, **kwargs):
    ns = "pointcloud_preprocessor"
    pkg = "autoware_pointcloud_preprocessor"

    separate_concatenate_node_and_time_sync_node = LaunchConfiguration(
        "separate_concatenate_node_and_time_sync_node"
    ).perform(context)
    is_separate_concatenate_node_and_time_sync_node = (
        separate_concatenate_node_and_time_sync_node.lower() == "true"
    )

    ## ---------------------- Set up Example parameters ---------------------------
    ## you could modify thie part and change the parameter directly in .yaml file.
    filter_param = load_parameter_dic("pointcloud_preprocessor", "config/filter_param_file.yaml")

    sync_and_concat_param = load_parameter_dic(
        "pointcloud_preprocessor", "config/concatenate_and_time_sync_param_file.yaml"
    )

    time_sync_param = load_parameter_dic(
        "pointcloud_preprocessor", "config/time_synchronizer_param_file.yaml"
    )

    concat_param = load_parameter_dic(
        "pointcloud_preprocessor", "config/concatenate_pointclouds_param_file.yaml"
    )

    crop_box_param = load_parameter_dic(
        "pointcloud_preprocessor", "config/crop_box_filter_param_file.yaml"
    )

    sync_and_concat_updated_param = {
        "input_topics": LaunchConfiguration("input_points_raw_list"),
        "output_frame": LaunchConfiguration("tf_output_frame"),
        "approximate_sync": True,
        "publish_synchronized_pointcloud": False,
    }
    updated_param = {
        "input_topics": LaunchConfiguration("input_points_raw_list"),
        "output_frame": LaunchConfiguration("tf_output_frame"),
        "approximate_sync": True,
    }
    crop_box_updated_param = {
        "input_frame": LaunchConfiguration("tf_output_frame"),
        "output_frame": LaunchConfiguration("tf_output_frame"),
        "min_x": -200.0,
        "max_x": 1000.0,
        "min_y": -50.0,
        "max_y": 50.0,
        "min_z": -2.0,
        "max_z": 3.0,
        "negative": False,
    }

    sync_and_concat_updated = {**filter_param, **sync_and_concat_param}
    sync_and_concat_updated.update(sync_and_concat_updated_param)

    time_sync_param.update(updated_param)

    concat_param_updated = {**filter_param, **concat_param}
    concat_param_updated.update(updated_param)

    crop_box_updated = {**filter_param, **crop_box_param}
    crop_box_updated.update(crop_box_updated_param)
    ## ----------------------  --------------------------  ---------------------------

    if not is_separate_concatenate_node_and_time_sync_node:
        sync_and_concat_component = ComposableNode(
            package=pkg,
            plugin="autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="sync_and_concatenate_filter",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "points_raw/concatenated"),
            ],
            parameters=[sync_and_concat_updated],
        )
        concat_components = [sync_and_concat_component]
    else:
        time_sync_component = ComposableNode(
            package=pkg,
            plugin="autoware::pointcloud_preprocessor::PointCloudDataSynchronizerComponent",
            name="synchronizer_filter",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "points_raw/concatenated"),
            ],
            parameters=[time_sync_param],
        )

        concat_component = ComposableNode(
            package=pkg,
            plugin="autoware::pointcloud_preprocessor::PointCloudConcatenationComponent",
            name="concatenate_filter",
            remappings=[("output", "points_raw/concatenated")],
            parameters=[concat_param_updated],
        )
        concat_components = [time_sync_component, concat_component]

    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package=pkg,
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            (
                "input",
                PythonExpression(
                    [
                        "'points_raw/concatenated' if len(",
                        LaunchConfiguration("input_points_raw_list"),
                        ") > 1 else ",
                        LaunchConfiguration("input_points_raw_list"),
                        "[0]",
                    ]
                ),
            ),
            ("output", LaunchConfiguration("output_points_raw")),
        ],
        parameters=[crop_box_updated],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=concat_components + [cropbox_component],
        output="screen",
    )

    # check the size of input_points_raw_list
    log_info = LogInfo(
        msg=PythonExpression(
            [
                "'input_points_raw_list size = ' + str(len(",
                LaunchConfiguration("input_points_raw_list"),
                "))",
            ]
        )
    )
    return [container, log_info]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "input_points_raw_list",
        ["/points_raw"],
        "Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )
    add_launch_arg("output_points_raw", "/points_raw/cropbox/filtered")
    add_launch_arg("tf_output_frame", "base_link")
    add_launch_arg(
        "separate_concatenate_node_and_time_sync_node",
        "true",
        "Set True to separate concatenate node and time_sync node. which will cause to larger memory usage.",
    )

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
