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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # autoware_local_map_provider executable
            Node(
                package="autoware_local_map_provider",
                executable="autoware_local_map_provider_exe",
                name="autoware_local_map_provider",
                namespace="mapless_architecture",
                remappings=[
                    (
                        "local_map_provider_node/output/local_map",
                        "local_map_provider_node/output/local_map",
                    ),
                    (
                        "local_map_provider_node/input/road_segments",
                        "local_road_provider_node/output/road_segments",
                    ),
                ],
                parameters=[],
                output="screen",
            ),
        ]
    )
