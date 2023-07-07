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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def launch_ndt_scan_matcher():
    launch_path = os.path.join(get_package_share_directory("ndt_scan_matcher"), "launch/ndt_scan_matcher.launch.xml")
    ndt_scan_matcher_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([launch_path]),
        launch_arguments={
            'input_map_points_topic': '/map/pointcloud_map',
            'input/pointcloud': '/localization/util/downsample/pointcloud',
            'input_initial_pose_topic': '/localization/pose_twist_fusion_filter/biased_pose_with_covariance',
            'output_pose_topic': '/localization/pose_estimator/pose',
            'output_pose_with_covariance_topic': '/localization/pose_estimator/pose_with_covariance',
            'param_file': LaunchConfiguration("ndt_scan_matcher_param_path")
        }.items()
    )
    return ndt_scan_matcher_launcher

def launch_yabloc():
    # メモ：ここはできればtier4_localization_launch以下ではなくyabloc以下にあるlaunchを直接呼びに行ったほうが、上との並列性が出てわかりやすい気がする
    launch_path = os.path.join(get_package_share_directory("tier4_localization_launch"), "launch/pose_estimator/yabloc.launch.xml")
    ndt_scan_matcher_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([launch_path]),
    )
    return ndt_scan_matcher_launcher

def launch_gyro_odometer():
    launch_path = os.path.join(get_package_share_directory("gyro_odometer"), "launch/gyro_odometer.launch.xml")
    gyro_odometer_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([launch_path]),
        launch_arguments={
            'input_vehicle_twist_with_covariance_topic': '/sensing/vehicle_velocity_converter/twist_with_covariance',
            'output_twist_with_covariance_topic': '/localization/twist_estimator/twist_with_covariance',
            'output_twist_with_covariance_raw_topic': '/localization/twist_estimator/twist_with_covariance_raw',
        }.items()
    )
    return gyro_odometer_launcher


def launch_setup(context, *args, **kwargs):
    pose_sources = LaunchConfiguration("pose_sources").perform(context).split(',')
    twist_sources = LaunchConfiguration("twist_sources").perform(context).split(',')
    pose_twist_sources = LaunchConfiguration("pose_twist_sources").perform(context).split(',')

    # Namespace: /localization/pose_estimator
    pose_group_actions = [PushRosNamespace('pose_estimator')]
    for source in pose_sources:
        if source == 'lidar':
            pose_group_actions.append(launch_ndt_scan_matcher())
        elif source == 'camera':
            pose_group_actions.append(launch_yabloc())

    # Namespace: /localization/twist_estimator
    twist_group_actions = [PushRosNamespace('twist_estimator')]
    for source in twist_sources:
        if source == 'gyro_odom':
            twist_group_actions.append(launch_gyro_odometer())
        elif source == 'imu_odom_gnss':
            # Launch Eagleye with twist
            raise NotImplementedError

    # Namespace: /localization/pose_twist_estimator
    pose_twist_group_actions = [PushRosNamespace('pose_twist_estimator')]
    for source in pose_twist_sources:
        if source == 'imu_odom_gnss':
            # Launch Eagleye with pose+twist
            raise NotImplementedError

    return [
        GroupAction(pose_group_actions),
        GroupAction(twist_group_actions),
        GroupAction(pose_twist_group_actions)
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("pose_sources", "lidar", "TO BE FILLED"),
    add_launch_arg("twist_sources", "gyro_odom", "TO BE FILLED"),
    add_launch_arg("pose_twist_sources", "imu_odom", "TO BE FILLED"),

    ### TO BE DELETED ### 
    add_launch_arg(
        "ndt_scan_matcher_param_path",
        [
            FindPackageShare("ndt_scan_matcher"),
            "/config/ndt_scan_matcher.param.yaml",
        ],
        "path to ndt_scan_matcher param file",
    ),
    ### TO BE DELETED ### 


    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
