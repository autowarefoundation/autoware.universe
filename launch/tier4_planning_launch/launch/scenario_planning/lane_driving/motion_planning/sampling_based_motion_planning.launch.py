# Copyright 2021 Tier IV, Inc. All rights reserved.
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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # vehicle information param path
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # planning common param path
    common_param_path = os.path.join(
        get_package_share_directory("tier4_planning_launch"),
        "config",
        "scenario_planning",
        "common",
        "common.param.yaml",
    )
    with open(common_param_path, "r") as f:
        common_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # surround obstacle checker
    surround_obstacle_checker_param_path = os.path.join(
        get_package_share_directory("tier4_planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "motion_planning",
        "surround_obstacle_checker",
        "surround_obstacle_checker.param.yaml",
    )
    with open(surround_obstacle_checker_param_path, "r") as f:
        surround_obstacle_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    surround_obstacle_checker_component = ComposableNode(
        package="surround_obstacle_checker",
        plugin="surround_obstacle_checker::SurroundObstacleCheckerNode",
        name="surround_obstacle_checker",
        namespace="",
        remappings=[
            ("~/output/no_start_reason", "/planning/scenario_planning/status/no_start_reason"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
            (
                "~/output/velocity_limit_clear_command",
                "/planning/scenario_planning/clear_velocity_limit",
            ),
            (
                "~/input/pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/odometry", "/localization/kinematic_state"),
        ],
        parameters=[
            surround_obstacle_checker_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    sampler_node_param_path = os.path.join(
        get_package_share_directory("sampler_node"),
        "config",
        "sampler_node.param.yaml",
    )
    with open(sampler_node_param_path, "r") as f:
        sampler_node_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    sampler_node = Node(
        package="sampler_node",
        # plugin='sampler_node::TrajectorySamplerNode',
        executable="trajectory_sampler_node_exe",
        name="trajectory_sampler_node",
        namespace="",
        remappings=[
            # ("~/output/trajectory", "sampling_planner/trajectory"),
            # ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
            ("~/output/trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/steer", "/vehicle/status/steering_status"),
            ("~/input/path", LaunchConfiguration("input_path_topic")),
            ("~/input/vector_map", LaunchConfiguration("input_map_topic")),
            ("~/input/route", LaunchConfiguration("input_route_topic")),
            ("~/input/fallback", "obstacle_avoidance_planner/trajectory"),
        ],
        parameters=[
            common_param,
            vehicle_info_param,
            sampler_node_param,
        ],
        prefix=["konsole -e gdb -ex run --args"],  # for debugging
        # prefix=['valgrind --tool=callgrind'],  # for profiling
        # prefix=['valgrind --leak-check=full \
        #  --show-leak-kinds=all \
        #  --track-origins=yes \
        #  --verbose \
        #  --log-file=valgrind-out.txt'],
        # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="motion_planning_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
    )

    surround_obstacle_checker_loader = LoadComposableNodes(
        composable_node_descriptions=[surround_obstacle_checker_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("use_surround_obstacle_check")),
    )

    group = GroupAction(
        [
            container,
            surround_obstacle_checker_loader,
            sampler_node,
        ]
    )
    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # vehicle information parameter file
    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    # obstacle_avoidance_planner
    add_launch_arg(
        "input_path_topic",
        "/planning/scenario_planning/lane_driving/behavior_planning/path",
        "input path topic of obstacle_avoidance_planner",
    )

    # surround obstacle checker
    add_launch_arg("use_surround_obstacle_check", "true", "launch surround_obstacle_checker or not")

    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")
    add_launch_arg("input_map_topic", "/map/vector_map", "vector map topic"),
    add_launch_arg("input_route_topic", "/planning/mission_planning/route", "route topic"),

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
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
