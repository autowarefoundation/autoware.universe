# Copyright 2019 Canonical, Ltd.
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
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    parameters_file = os.path.join(
        get_package_share_directory('mouse_teleop'),
        'config', 'mouse_teleop.yaml'
    )

    mouse_teleop = launch_ros.actions.Node(
            package='mouse_teleop', executable='mouse_teleop',
            parameters=[parameters_file])

    return LaunchDescription([mouse_teleop])
