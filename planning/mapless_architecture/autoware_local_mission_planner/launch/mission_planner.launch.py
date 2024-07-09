# Copyright 2024 driveblocks GmbH, authors: Simon Eisenmann, Thomas Herrmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_planner_param_file = os.path.join(
        get_package_share_directory("autoware_local_mission_planner"),
        "param",
        "mission_planner_default.yaml",
    )

    mission_planner_param = DeclareLaunchArgument(
        "mission_planner_param_file",
        default_value=mission_planner_param_file,
        description="Path to config file for the mission planner.",
    )

    return LaunchDescription(
        [
            # mission planner parameters
            mission_planner_param,
            # mission_planner executable
            Node(
                package="autoware_local_mission_planner",
                executable="autoware_local_mission_planner_exe",
                name="autoware_local_mission_planner",
                namespace="mapless_architecture",
                remappings=[
                    (
                        "mission_planner_node/output/marker",
                        "mission_planner_node/output/marker",
                    ),
                    (
                        "mission_planner_node/output/mission_lanes_stamped",
                        "mission_planner_node/output/mission_lanes_stamped",
                    ),
                    (
                        "mission_planner_node/input/local_map",
                        "local_map_provider_node/output/local_map",
                    ),
                    ("mission_planner/input/mission", "hmi_node/output/mission"),
                    (
                        "mission_planner/input/state_estimate",
                        "/awsim/ground_truth/localization/kinematic_state",
                    ),
                ],
                parameters=[
                    LaunchConfiguration("mission_planner_param_file"),
                ],
                output="screen",
            ),
        ]
    )
