# Copyright 2021 The Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.


import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch freespace_planner with default configuration."""
    # -------------------------------- Nodes-----------------------------------
    freespace_planner_node = launch_ros.actions.Node(
        package='freespace_planner',
        executable='freespace_planner_node_exe',
        name='freespace_planner',
        namespace='planning',
        output='screen',
        parameters=[
            "{}/param/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "freespace_planner"
                )
            ),
        ]
    )

    ld = launch.LaunchDescription([freespace_planner_node])
    return ld
