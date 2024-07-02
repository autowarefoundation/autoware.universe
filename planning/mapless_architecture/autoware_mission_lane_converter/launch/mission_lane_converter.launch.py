# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_lane_converter_param_file = os.path.join(
        get_package_share_directory("autoware_mission_lane_converter"),
        "param",
        "mission_lane_converter_default.yaml",
    )

    mission_lane_converter_param = DeclareLaunchArgument(
        "mission_lane_converter_param_file",
        default_value=mission_lane_converter_param_file,
        description="Path to config file for the mission lane converter.",
    )

    return LaunchDescription(
        [
            # mission lane converter parameter
            mission_lane_converter_param,
            # mission lane converter executable
            Node(
                package="autoware_mission_lane_converter",
                executable="autoware_mission_lane_converter",
                name="autoware_mission_lane_converter",
                namespace="mapless_architecture",
                remappings=[
                    (
                        "mission_lane_converter/input/mission_lanes",
                        "mission_planner_node/output/mission_lanes_stamped",
                    ),
                    (
                        "mission_lane_converter/input/odometry",
                        "/awsim/ground_truth/localization/kinematic_state",
                    ),
                    (
                        "mission_lane_converter/output/trajectory",
                        "/planning/scenario_planning/local_trajectory",
                    ),
                    (
                        "mission_lane_converter/output/global_trajectory",
                        "/planning/scenario_planning/trajectory",
                    ),
                    (
                        "mission_lane_converter/output/path",
                        "mission_lane_converter/output/local_path",
                    ),
                    (
                        "mission_lane_converter/output/global_path",
                        "mission_lane_converter/output/global_path",
                    ),
                ],
                parameters=[
                    LaunchConfiguration("mission_lane_converter_param_file"),
                ],
                output="screen",
            ),
        ]
    )
