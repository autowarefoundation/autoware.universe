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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
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


def create_additional_pipeline(vehicle_info, lidar_name):
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
                "min_x": -50.0,
                "max_x": 100.0,
                "min_y": -50.0,
                "max_y": 50.0,
                "min_z": vehicle_info["min_height_offset"],
                "max_z": vehicle_info["max_height_offset"],
                "negative": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name=f"{lidar_name}_scan_ground_filter",
        remappings=[
            ("input", f"{lidar_name}/measurement_range_cropped/pointcloud"),
            ("output", f"{lidar_name}/no_ground/pointcloud"),
        ],
        parameters=[
            {
                "global_slope_max_angle_deg": 10.0,
                "local_slope_max_angle_deg": 30.0,
                "split_points_distance_tolerance": 0.1,
                "split_height_distance": 0.05,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    return [crop_box_filter_component, ground_filter_component]


def launch_setup(context, *args, **kwargs):

    vehicle_info = get_vehicle_info(context)

    path = LaunchConfiguration("perception_config_file").perform(context)
    with open(path, "r") as f:
        pipeline_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    additional_pipeline_components = []
    for lidar_name in pipeline_param["additional_lidars"]:
        additional_pipeline_components.extend(create_additional_pipeline(vehicle_info, lidar_name))

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
                "min_x": -50.0,
                "max_x": 100.0,
                "min_y": -50.0,
                "max_y": 50.0,
                "min_z": vehicle_info["min_height_offset"],
                "max_z": vehicle_info["max_height_offset"],
                "negative": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_filter_component = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name="scan_ground_filter",
        remappings=[
            ("input", "measurement_range_cropped/pointcloud"),
            ("output", "no_ground/pointcloud"),
        ],
        parameters=[
            {
                "global_slope_max_angle_deg": 10.0,
                "local_slope_max_angle_deg": 30.0,
                "split_points_distance_tolerance": 0.2,
                "split_height_distance": 0.2,
                "use_virtual_ground_point": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ground_concat_topics = ["no_ground/pointcloud"]
    for lidar_name in pipeline_param["additional_lidars"]:
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

    laserscan_to_occupancy_grid_map_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("laserscan_to_occupancy_grid_map"),
                "/launch/laserscan_to_occupancy_grid_map.launch.py",
            ]
        ),
        launch_arguments={
            "container": "/perception/object_segmentation/ground_segmentation/perception_pipeline_container",
            "input/obstacle_pointcloud": "no_ground/oneshot/pointcloud"
            if bool(pipeline_param["additional_lidars"])
            else "no_ground/pointcloud",
            "input/raw_pointcloud": "/sensing/lidar/concatenated/pointcloud",
            "output": "occupancy_grid",
            "use_intra_process": LaunchConfiguration("use_intra_process"),
        }.items(),
    )

    occupancy_grid_map_outlier_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::OccupancyGridMapOutlierFilterComponent",
        name="occupancy_grid_map_outlier_filter",
        remappings=[
            ("~/input/occupancy_grid_map", "occupancy_grid"),
            (
                "~/input/pointcloud",
                "no_ground/oneshot/pointcloud"
                if bool(pipeline_param["additional_lidars"])
                else "no_ground/pointcloud",
            ),
            ("~/output/pointcloud", "/perception/object_segmentation/pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="perception_pipeline_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            crop_box_filter_component,
            ground_filter_component,
            occupancy_grid_map_outlier_component,
        ],
        output="screen",
    )

    additional_pipeline_loader = LoadComposableNodes(
        composable_node_descriptions=additional_pipeline_components,
        target_container=container,
        condition=IfCondition(
            LaunchConfiguration(
                "use_additional_pipeline", default=bool(pipeline_param["additional_lidars"])
            )
        ),
    )

    concat_data_component_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_data_component],
        target_container=container,
        condition=IfCondition(
            LaunchConfiguration(
                "use_additional_pipeline", default=bool(pipeline_param["additional_lidars"])
            )
        ),
    )

    return [
        container,
        additional_pipeline_loader,
        concat_data_component_loader,
        laserscan_to_occupancy_grid_map_loader,
    ]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("vehicle_param_file")
    add_launch_arg("perception_config_file")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")

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
