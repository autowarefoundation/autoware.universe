# Copyright 2021 Tier IV, Inc. All rights reserved.
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
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    pose_sources = LaunchConfiguration("pose_sources").perform(context).split(',')

    ndt_enabled = 'true' if 'lidar' in pose_sources else 'false'
    yabloc_enabled = 'true' if 'camera' in pose_sources else 'false'

    if LaunchConfiguration("system_run_mode").perform(context) == 'online':
        stop_check_enabled = 'false'
    elif LaunchConfiguration("system_run_mode").perform(context) == 'logging_simulation':
        stop_check_enabled = 'true'
    else:
        raise NotImplementedError

    launch_path = os.path.join(get_package_share_directory("pose_initializer"), "launch/pose_initializer.launch.xml")
    pose_initializer_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([launch_path]),
        launch_arguments={
            'gnss_enabled': 'true',
            'ndt_enabled': ndt_enabled,
            'ekf_enabled': 'true',
            'yabloc_enabled': yabloc_enabled,
            'stop_check_enabled': stop_check_enabled,
            'param_file': LaunchConfiguration("pose_initializer_param_path"),
            'sub_gnss_pose_cov': '/sensing/gnss/pose_with_covariance',
        }.items()
    )

    return [
        pose_initializer_launcher
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("pose_sources", "lidar", "TO BE FILLED"),
    add_launch_arg("system_run_mode", "", "TO BE FILLED"),
    add_launch_arg(
        "pose_initializer_param_path",
        [
            FindPackageShare("pose_estimator"),
            "/config/pose_initializer.param.yaml",
        ],
        "path to pose_estimator param file",
    ),

    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
