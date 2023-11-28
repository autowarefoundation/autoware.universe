# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('autodrive_f1tenth'), 'config/gym_rviz', 'perception.yaml')

    gym_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('autodrive_f1tenth'), 'launch/gym_rviz'),
            '/gym_rviz_bringup.launch.py']),
        launch_arguments = {'default_rviz' : 'False'}.items(),
    )

    perception_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[config]
    )

    planning_node = Node(
        package="recordreplay_planner_nodes",
        executable="recordreplay_planner_node_exe",
        name="recordreplay_planner",
        namespace="planning",
        output="screen",
        emulate_tty=True,
        parameters=["{}/config/gym_rviz/planning.yaml".format(get_package_share_directory('autodrive_f1tenth')),
        ],
    )

    control_node = Node(
        package="autodrive_trajectory_follower",
        executable="autodrive_trajectory_follower_exe",
        name="autodrive_trajectory_follower",
        output='screen',
        emulate_tty=True,
        parameters=["{}/config/gym_rviz/control.yaml".format(get_package_share_directory('autodrive_f1tenth')),
        ],
        remappings=[
            ("input/kinematics", "/ego_racecar/odom"), # /particle_filter/pose/odom
            ("input/trajectory", "/planning/trajectory"),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('autodrive_f1tenth'), 'rviz/gym_rviz', 'replay.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(gym_rviz_launch)
    ld.add_action(perception_node)
    ld.add_action(planning_node)
    ld.add_action(control_node)
    ld.add_action(rviz_node)
    return ld