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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    mission_planner_launch_file = "/mission_planner.launch.py"

    mission_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), mission_planner_launch_file]),
        # launch_arguments={'node_name': 'bar'}.items()
    )

    # workaround due to ros2 bug with using ThisLaunchFileDir()
    # https://github.com/ros2/launch/issues/618
    # https://gitlab.com/driveblocks/mod_feature_detection/-/merge_requests/102/diffs
    mission_lane_converter_pkg_share_directory = get_package_share_directory(
        "autoware_mission_lane_converter"
    )
    mission_lane_converter_file_name = "mission_lane_converter.launch.py"
    mission_lane_converter_path = (
        Path(mission_lane_converter_pkg_share_directory)
        / "launch/"
        / mission_lane_converter_file_name
    )

    mission_lane_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mission_lane_converter_path)),
        # launch_arguments={'node_name': 'bar'}.items()
    )

    hmi_pkg_share_directory = get_package_share_directory("autoware_hmi")
    hmi_file_name = "hmi.launch.py"
    hmi_path = Path(hmi_pkg_share_directory) / "launch/" / hmi_file_name

    hmi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(hmi_path)),
        # launch_arguments={'node_name': 'bar'}.items()
    )

    local_map_provider_pkg_share_directory = get_package_share_directory(
        "autoware_local_map_provider"
    )
    local_map_provider_file_name = "local_map_provider.launch.py"
    local_map_provider_path = (
        Path(local_map_provider_pkg_share_directory) / "launch/" / local_map_provider_file_name
    )

    local_map_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(local_map_provider_path)),
        # launch_arguments={'node_name': 'bar'}.items()
    )

    return LaunchDescription([mission_planner, mission_lane_converter, hmi, local_map_provider])
