# Copyright 2022 TIER IV, Inc.
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
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics0",
                executable="image_diagnostics_node",
                name="image_diagnostics_node0",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera0/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image0"),
                ],
            ),
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics1",
                executable="image_diagnostics_node",
                name="image_diagnostics_node1",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera1/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image1"),
                ],
            ),
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics2",
                executable="image_diagnostics_node",
                name="image_diagnostics_node2",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera2/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image2"),
                ],
            ),
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics3",
                executable="image_diagnostics_node",
                name="image_diagnostics_node3",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera3/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image3"),
                ],
            ),
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics4",
                executable="image_diagnostics_node",
                name="image_diagnostics_node4",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera4/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image4"),
                ],
            ),
            Node(
                package="image_diagnostics",
                namespace="image_diagnostics5",
                executable="image_diagnostics_node",
                name="image_diagnostics_node5",
                remappings=[
                    (
                        "input/compressed_image",
                        "/sensing/camera/camera5/image_rect_color/compressed",
                    ),
                    ("image_diag/raw_image", "image_diag/raw_image5"),
                ],
            ),
        ]
    )
