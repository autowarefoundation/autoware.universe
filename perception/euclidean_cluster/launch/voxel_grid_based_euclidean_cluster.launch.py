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
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution
from launch.substitutions import AnonName
from launch.substitutions import LaunchConfiguration
from launch.substitutions import NotSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # https://github.com/ros2/launch_ros/issues/156
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    ns = ""
    pkg = "euclidean_cluster"

    # set compare map filter as a component
    compare_map_filter_component = ComposableNode(
        package="compare_map_segmentation",
        namespace=ns,
        plugin="compare_map_segmentation::VoxelBasedCompareMapFilterComponent",
        name=AnonName("compare_map_filter"),
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("map", LaunchConfiguration("input_map")),
            ("output", "map_filter/pointcloud"),
            ("map_loader_service", "/map/get_differential_pointcloud_map"),
            ("kinematic_state", "/localization/kinematic_state"),
        ],
        parameters=[load_composable_node_param("compare_map_param_path")],
    )

    # separate range of pointcloud when map_filter used
    use_map_short_range_crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="short_distance_crop_box_range",
        remappings=[
            ("input", "map_filter/pointcloud"),
            ("output", "short_range/pointcloud"),
        ],
        parameters=[
            load_composable_node_param("voxel_grid_based_euclidean_param_path"),
            {
                "negative": False,
            },
        ],
    )

    use_map_long_range_crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="long_distance_crop_box_range",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", "long_range/pointcloud"),
        ],
        parameters=[
            load_composable_node_param("voxel_grid_based_euclidean_param_path"),
            {
                "negative": True,
            },
        ],
    )

    # disuse_map_voxel_grid_filter
    disuse_map_short_range_crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="short_distance_crop_box_range",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", "short_range/pointcloud"),
        ],
        parameters=[
            load_composable_node_param("voxel_grid_based_euclidean_param_path"),
            {
                "negative": False,
            },
        ],
    )

    disuse_map_long_range_crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="long_distance_crop_box_range",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", "long_range/pointcloud"),
        ],
        parameters=[
            load_composable_node_param("voxel_grid_based_euclidean_param_path"),
            {
                "negative": True,
            },
        ],
    )

    # set outlier filter as a component
    outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::VoxelGridOutlierFilterComponent",
        name="outlier_filter",
        remappings=[
            ("input", "short_range/pointcloud"),
            ("output", "outlier_filter/pointcloud"),
        ],
        parameters=[load_composable_node_param("outlier_param_path")],
    )

    # concat with-outlier pointcloud and without-outlier pcl
    downsample_concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        namespace=ns,
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concat_downsampled_pcl",
        remappings=[("output", "downsampled/concatenated/pointcloud")],
        parameters=[
            {
                "input_topics": ["long_range/pointcloud", "outlier_filter/pointcloud"],
                "output_frame": "base_link",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # set euclidean cluster as a component
    use_downsample_euclidean_cluster_component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", "downsampled/concatenated/pointcloud"),
            ("output", LaunchConfiguration("output_clusters")),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )

    use_map_disuse_downsample_euclidean_component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", "map_filter/pointcloud"),
            ("output", LaunchConfiguration("output_clusters")),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )
    disuse_map_disuse_downsample_euclidean_component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", LaunchConfiguration("output_clusters")),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )

    container = ComposableNodeContainer(
        name="euclidean_cluster_container",
        package="rclcpp_components",
        namespace=ns,
        executable="component_container",
        composable_node_descriptions=[],
        output="screen",
    )

    use_map_use_downsample_loader = LoadComposableNodes(
        composable_node_descriptions=[
            compare_map_filter_component,
            use_map_short_range_crop_box_filter_component,
            use_map_long_range_crop_box_filter_component,
            outlier_filter_component,
            downsample_concat_component,
            use_downsample_euclidean_cluster_component,
        ],
        target_container=container,
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("use_pointcloud_map"),
                LaunchConfiguration("use_downsample_pointcloud"),
            )
        ),
    )

    disuse_map_use_downsample_loader = LoadComposableNodes(
        composable_node_descriptions=[
            disuse_map_short_range_crop_box_filter_component,
            disuse_map_long_range_crop_box_filter_component,
            outlier_filter_component,
            downsample_concat_component,
            use_downsample_euclidean_cluster_component,
        ],
        target_container=container,
        condition=IfCondition(
            AndSubstitution(
                NotSubstitution(LaunchConfiguration("use_pointcloud_map")),
                LaunchConfiguration("use_downsample_pointcloud"),
            )
        ),
    )

    use_map_disuse_downsample_loader = LoadComposableNodes(
        composable_node_descriptions=[
            compare_map_filter_component,
            use_map_disuse_downsample_euclidean_component,
        ],
        target_container=container,
        condition=IfCondition(
            AndSubstitution(
                (LaunchConfiguration("use_pointcloud_map")),
                NotSubstitution(LaunchConfiguration("use_downsample_pointcloud")),
            )
        ),
    )

    disuse_map_disuse_downsample_loader = LoadComposableNodes(
        composable_node_descriptions=[
            disuse_map_disuse_downsample_euclidean_component,
        ],
        target_container=container,
        condition=IfCondition(
            AndSubstitution(
                NotSubstitution(LaunchConfiguration("use_pointcloud_map")),
                NotSubstitution(LaunchConfiguration("use_downsample_pointcloud")),
            )
        ),
    )
    return [
        container,
        use_map_use_downsample_loader,
        disuse_map_use_downsample_loader,
        use_map_disuse_downsample_loader,
        disuse_map_disuse_downsample_loader,
    ]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("input_pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            add_launch_arg("input_map", "/map/pointcloud_map"),
            add_launch_arg("output_clusters", "clusters"),
            add_launch_arg("use_pointcloud_map", "false"),
            add_launch_arg("use_downsample_pointcloud", "false"),
            add_launch_arg(
                "voxel_grid_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/object_recognition/detection/clustering/voxel_grid.param.yaml",
                ],
            ),
            add_launch_arg(
                "outlier_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/object_recognition/detection/clustering/outlier.param.yaml",
                ],
            ),
            add_launch_arg(
                "compare_map_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/object_recognition/detection/clustering/compare_map.param.yaml",
                ],
            ),
            add_launch_arg(
                "voxel_grid_based_euclidean_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
