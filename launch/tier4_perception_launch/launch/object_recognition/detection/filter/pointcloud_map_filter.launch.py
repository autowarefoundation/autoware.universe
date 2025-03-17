# Copyright 2022 TIER IV, Inc. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


class PointcloudMapFilterPipeline:
    def __init__(self, context):
        pointcloud_map_filter_param_path = os.path.join(
            LaunchConfiguration(
                "object_recognition_detection_pointcloud_map_filter_param_path"
            ).perform(context),
        )
        with open(pointcloud_map_filter_param_path, "r") as f:
            self.pointcloud_map_filter_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        self.voxel_size = self.pointcloud_map_filter_param["down_sample_voxel_size"]
        self.use_pointcloud_map = LaunchConfiguration("use_pointcloud_map").perform(context)

    def create_pipeline(self):
        if self.use_pointcloud_map == "true":
            return self.create_compare_map_pipeline()
        else:
            return self.create_no_compare_map_pipeline()

    def create_no_compare_map_pipeline(self):
        components = []
        components.append(
            ComposableNode(
                package="autoware_pointcloud_preprocessor",
                plugin="autoware::pointcloud_preprocessor::ApproximateDownsampleFilterComponent",
                name="voxel_grid_downsample_filter",
                remappings=[
                    ("input", LaunchConfiguration("input_topic")),
                    ("output", LaunchConfiguration("output_topic")),
                ],
                parameters=[
                    {
                        "voxel_size_x": self.voxel_size,
                        "voxel_size_y": self.voxel_size,
                        "voxel_size_z": self.voxel_size,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        )
        return components

    def create_compare_map_pipeline(self):
        components = []
        down_sample_topic = (
            "/perception/obstacle_segmentation/pointcloud_map_filtered/downsampled/pointcloud"
        )
        components.append(
            ComposableNode(
                package="autoware_pointcloud_preprocessor",
                plugin="autoware::pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
                name="voxel_grid_downsample_filter",
                remappings=[
                    ("input", LaunchConfiguration("input_topic")),
                    ("output", down_sample_topic),
                ],
                parameters=[
                    {
                        "voxel_size_x": self.voxel_size,
                        "voxel_size_y": self.voxel_size,
                        "voxel_size_z": self.voxel_size,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        )
        components.append(
            ComposableNode(
                package="autoware_compare_map_segmentation",
                plugin="autoware::compare_map_segmentation::VoxelBasedCompareMapFilterComponent",
                name="voxel_based_compare_map_filter",
                remappings=[
                    ("input", down_sample_topic),
                    ("map", "/map/pointcloud_map"),
                    ("output", LaunchConfiguration("output_topic")),
                    ("map_loader_service", "/map/get_differential_pointcloud_map"),
                    ("kinematic_state", "/localization/kinematic_state"),
                ],
                parameters=[
                    self.pointcloud_map_filter_param,
                    {
                        "input_frame": "map",
                        "use_pc_stamp_for_dynamic_transform_lookup": True,
                    },
                ],
                extra_arguments=[
                    {"use_intra_process_comms": False},
                ],
            )
        )
        return components


def launch_setup(context, *args, **kwargs):
    pipeline = PointcloudMapFilterPipeline(context)
    components = []
    components.extend(pipeline.create_pipeline())
    pointcloud_container_loader = LoadComposableNodes(
        composable_node_descriptions=components,
        target_container=LaunchConfiguration("pointcloud_container_name"),
    )
    return [pointcloud_container_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("input_topic", "")
    add_launch_arg("output_topic", "")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("use_pointcloud_map", "true")
    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )
    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
