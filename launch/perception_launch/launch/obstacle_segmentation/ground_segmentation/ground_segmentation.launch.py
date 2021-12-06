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
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(context):
    path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
    p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
    p["min_longitudinal_offset"] = -p["rear_overhang"]
    p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
    p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
    p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = p["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)
    return p


def create_additional_pipeline(vehicle_info, lidar_name, ground_segmentation_param):
    crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name=f"{lidar_name}_crop_box_filter",
        remappings=[
            ("input", f"/sensing/lidar/{lidar_name}/outlier_filtered/pointcloud"),
            ("output", f"{lidar_name}/measurement_range_cropped/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "min_z": vehicle_info["min_height_offset"],
                "max_z": vehicle_info["max_height_offset"],
            },
            ground_segmentation_param[f"{lidar_name}_crop_box_filter"]["parameters"],
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin=ground_segmentation_param[f"{lidar_name}_ground_filter"]["plugin"],
        name=f"{lidar_name}_ground_filter",
        remappings=[
            ("input", f"{lidar_name}/measurement_range_cropped/pointcloud"),
            ("output", f"{lidar_name}/no_ground/pointcloud"),
        ],
        parameters=[ground_segmentation_param[f"{lidar_name}_ground_filter"]["parameters"]],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [crop_box_filter_component, ground_filter_component]


def create_ransac_pipeline(ground_segmentation_param):
    livox_concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="livox_concatenate_data",
        remappings=[("output", "livox_concatenated/pointcloud")],
        parameters=[
            {
                "input_topics": [
                    "/sensing/lidar/front_left/min_range_cropped/pointcloud",
                    "/sensing/lidar/front_right/min_range_cropped/pointcloud",
                    "/sensing/lidar/front_center/min_range_cropped/pointcloud",
                ],
                "output_frame": LaunchConfiguration("base_frame"),
                "timeout_sec": 1.0,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    short_height_obstacle_detection_area_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="short_height_obstacle_detection_area_filter",
        remappings=[
            ("input", "livox_concatenated/pointcloud"),
            ("output", "short_height_obstacle_detection_area/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
            },
            ground_segmentation_param["short_height_obstacle_detection_area_filter"]["parameters"],
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    vector_map_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::Lanelet2MapFilterComponent",
        name="vector_map_filter",
        remappings=[
            ("input/pointcloud", "short_height_obstacle_detection_area/pointcloud"),
            ("input/vector_map", "/map/vector_map"),
            ("output", "vector_map_filtered/pointcloud"),
        ],
        parameters=[
            {
                "voxel_size_x": 0.25,
                "voxel_size_y": 0.25,
            }
        ],
        # cannot use intra process because vector map filter uses transient local.
        extra_arguments=[{"use_intra_process_comms": False}],
    )

    ransac_ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::RANSACGroundFilterComponent",
        name="ransac_ground_filter",
        remappings=[
            ("input", "vector_map_filtered/pointcloud"),
            ("output", "short_height/no_ground/pointcloud"),
        ],
        parameters=[ground_segmentation_param["ransac_ground_filter"]["parameters"]],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [
        livox_concat_component,
        short_height_obstacle_detection_area_filter_component,
        vector_map_filter_component,
        ransac_ground_filter_component,
    ]


def create_elevation_map_filter_pipeline():
    compare_elevation_map_filter_component = ComposableNode(
        package="compare_map_segmentation",
        plugin="compare_map_segmentation::CompareElevationMapFilterComponent",
        name="compare_elevation_map_filter",
        remappings=[
            ("input", "no_ground/oneshot/pointcloud"),
            ("output", "map_filtered/pointcloud"),
            ("input/elevation_map", "/map/elevation_map"),
        ],
        parameters=[
            {
                "map_frame": "map",
                "map_layer_name": "elevation",
                "height_diff_thresh": 0.15,
                "input_frame": "map",
                "output_frame": "base_link",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": False}],  # can't use this with transient_local
    )

    downsampling_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
        name="voxel_grid_filter",
        remappings=[
            ("input", "map_filtered/pointcloud"),
            ("output", "voxel_grid_filtered/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "voxel_size_x": 0.04,
                "voxel_size_y": 0.04,
                "voxel_size_z": 0.08,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    voxel_grid_outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::VoxelGridOutlierFilterComponent",
        name="voxel_grid_outlier_filter",
        remappings=[
            ("input", "voxel_grid_filtered/pointcloud"),
            ("output", "/perception/obstacle_segmentation/pointcloud"),
        ],
        parameters=[
            {
                "voxel_size_x": 0.4,
                "voxel_size_y": 0.4,
                "voxel_size_z": 100.0,
                "voxel_points_threshold": 5,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [
        compare_elevation_map_filter_component,
        downsampling_component,
        voxel_grid_outlier_filter_component,
    ]


def launch_setup(context, *args, **kwargs):

    vehicle_info = get_vehicle_info(context)

    ground_segmentation_param_path = os.path.join(
        get_package_share_directory("perception_launch"),
        "config/obstacle_segmentation/ground_segmentation/ground_segmentation.param.yaml",
    )
    with open(ground_segmentation_param_path, "r") as f:
        ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    additional_pipeline_components = []
    if ground_segmentation_param["use_ransac_pipeline"]:
        additional_pipeline_components = create_ransac_pipeline(ground_segmentation_param)
    else:
        for lidar_name in ground_segmentation_param["additional_lidars"]:
            additional_pipeline_components.extend(
                create_additional_pipeline(vehicle_info, lidar_name, ground_segmentation_param)
            )

    crop_box_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            ("input", "/sensing/lidar/concatenated/pointcloud"),
            ("output", "measurement_range_cropped/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "min_z": vehicle_info["min_height_offset"],
                "max_z": vehicle_info["max_height_offset"],
            },
            ground_segmentation_param["common_crop_box_filter"]["parameters"],
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin=ground_segmentation_param["common_ground_filter"]["plugin"],
        name="common_ground_filter",
        remappings=[
            ("input", "measurement_range_cropped/pointcloud"),
            ("output", "no_ground/pointcloud"),
        ],
        parameters=[ground_segmentation_param["common_ground_filter"]["parameters"]],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_concat_topics = ["no_ground/pointcloud"]
    for lidar_name in ground_segmentation_param["additional_lidars"]:
        ground_concat_topics.extend([f"{lidar_name}/no_ground/pointcloud"])

    concat_data_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[("output", "no_ground/oneshot/pointcloud")],
        parameters=[
            {
                "input_topics": ground_concat_topics,
                "output_frame": LaunchConfiguration("base_frame"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    occupancy_outlier_filter_component = ComposableNode(
        package="occupancy_grid_map_outlier_filter",
        plugin="occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent",
        name="occupancy_grid_map_outlier_filter",
        remappings=[
            ("~/input/occupancy_grid_map", "/perception/occupancy_grid_map/map"),
            (
                "~/input/pointcloud",
                "no_ground/oneshot/pointcloud"
                if bool(ground_segmentation_param["additional_lidars"])
                else "no_ground/pointcloud",
            ),
            ("~/output/pointcloud", "/perception/obstacle_segmentation/pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            crop_box_filter_component,
            ground_filter_component,
        ],
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    composable_nodes_loader = LoadComposableNodes(
        composable_node_descriptions=[
            crop_box_filter_component,
            ground_filter_component,
        ],
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    additional_pipeline_loader = LoadComposableNodes(
        composable_node_descriptions=additional_pipeline_components,
        target_container=target_container,
        condition=IfCondition(
            LaunchConfiguration(
                "use_additional_pipeline",
                default=bool(ground_segmentation_param["additional_lidars"]),
            )
        ),
    )

    concat_data_component_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_data_component],
        target_container=target_container,
        condition=IfCondition(
            LaunchConfiguration(
                "use_additional_pipeline",
                default=bool(ground_segmentation_param["additional_lidars"]),
            )
        ),
    )

    compare_map_component_loader = LoadComposableNodes(
        composable_node_descriptions=create_elevation_map_filter_pipeline(),
        target_container=target_container,
        condition=IfCondition(
            LaunchConfiguration(
                "use_compare_map_pipeline",
                default=ground_segmentation_param["use_compare_map_pipeline"],
            )
        ),
    )

    occupancy_grid_outlier_filter_component_loader = LoadComposableNodes(
        composable_node_descriptions=[occupancy_outlier_filter_component],
        target_container=target_container,
        condition=UnlessCondition(
            LaunchConfiguration(
                "use_compare_map_pipeline",
                default=ground_segmentation_param["use_compare_map_pipeline"],
            )
        ),
    )

    return [
        container,
        composable_nodes_loader,
        additional_pipeline_loader,
        compare_map_component_loader,
        concat_data_component_loader,
        occupancy_grid_outlier_filter_component_loader,
    ]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("vehicle_param_file")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "perception_pipeline_container")

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
