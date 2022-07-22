# Copyright 2021-2022 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch lidar segmentation node."""

import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    # lidar segmentation node execution definition.
    lidar_apollo_segmentation_tvm_node_runner = launch_ros.actions.Node(
        package="lidar_apollo_segmentation_tvm_nodes",
        executable="lidar_apollo_segmentation_tvm_nodes_exe",
        output="screen",
    )

    return launch.LaunchDescription([lidar_apollo_segmentation_tvm_node_runner])
