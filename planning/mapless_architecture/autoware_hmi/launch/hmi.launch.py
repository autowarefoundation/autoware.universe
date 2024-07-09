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
            # hmi executable
            Node(
                package="autoware_hmi",
                executable="autoware_hmi_exe",
                name="autoware_hmi",
                namespace="mapless_architecture",
                remappings=[
                    ("hmi_node/output/mission", "hmi_node/output/mission"),
                ],
                parameters=[],
                output="screen",
            ),
        ]
    )
