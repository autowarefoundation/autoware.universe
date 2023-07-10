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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
import os


def launch_pose_initializer(system_run_mode, ndt_enabled=False, yabloc_enabled=False):
    if system_run_mode == 'online':
        stop_check_enabled = 'false'
    elif system_run_mode == 'logging_simulation':
        stop_check_enabled = 'true'
    else:
        raise NotImplementedError

    pose_initializer_launch_path = os.path.join(get_package_share_directory("pose_initializer"), "launch/pose_initializer.launch.xml")
    pose_initializer_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([pose_initializer_launch_path]),
        launch_arguments={
            'gnss_enabled': 'true',
            'ndt_enabled': 'true' if ndt_enabled else 'false',
            'ekf_enabled': 'true',
            'yabloc_enabled': 'true' if yabloc_enabled else 'false',
            'stop_check_enabled': stop_check_enabled,
            'config_file': LaunchConfiguration("pose_initializer_param_path"),
            'sub_gnss_pose_cov': '/sensing/gnss/pose_with_covariance',
        }.items()
    )

    automatic_pose_initializer_launch_path = os.path.join(get_package_share_directory("automatic_pose_initializer"), "launch/automatic_pose_initializer.launch.xml")
    automatic_pose_initializer_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([automatic_pose_initializer_launch_path]),
    )
    return [pose_initializer_launcher, automatic_pose_initializer_launcher]

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

def launch_preprocessing_for_ndt():
    launch_path = os.path.join(get_package_share_directory("tier4_localization_launch"), "launch/util/util.launch.py")
    pcd_preprocessor_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_path]),
        launch_arguments={
            'input/pointcloud': LaunchConfiguration("input/pointcloud"),
            'output/pointcloud': LaunchConfiguration("output/pointcloud"),
            'use_pointcloud_container': LaunchConfiguration("use_pointcloud_container"),
            'pointcloud_container_name': LaunchConfiguration("pointcloud_container_name"),
            'use_intra_process': LaunchConfiguration("use_intra_process"),
        }.items()
    )
    return pcd_preprocessor_launcher

def launch_yabloc():
    launch_path = os.path.join(get_package_share_directory("tier4_localization_launch"), "launch/pose_estimator/yabloc.launch.xml")
    yabloc_launcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([launch_path]),
    )
    return yabloc_launcher

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

def launch_eagleye(mode):
    return None

def launch_setup(context, *args, **kwargs):
    pose_source = LaunchConfiguration("pose_source").perform(context)
    twist_source = LaunchConfiguration("twist_source").perform(context)
    system_run_mode = LaunchConfiguration("system_run_mode").perform(context)

    pose_group_actions = [PushRosNamespace('pose_estimator')]  # Namespace: /localization/pose_estimator
    twist_group_actions = [PushRosNamespace('twist_estimator')]  # Namespace: /localization/twist_estimator
    pose_twist_group_actions = [PushRosNamespace('pose_twist_estimator')]  # Namespace: /localization/pose_twist_estimator
    util_group_actions = [PushRosNamespace('util')]  # Namespace: /localization/util

    # ndt_scan_matcher: pose_estimator
    if 'lidar' == pose_source:
        pose_group_actions += [launch_ndt_scan_matcher()] # ndt_scan_matcher
        util_group_actions += launch_pose_initializer(system_run_mode, ndt_enabled=True, yabloc_enabled=False) # pose_initializer
        util_group_actions += [launch_preprocessing_for_ndt()] # pointcloud_preprocessor

    # yabloc: pose_estimator
    if 'camera' == pose_source:
        pose_group_actions += [launch_yabloc()] # yabloc
        util_group_actions += launch_pose_initializer(system_run_mode, ndt_enabled=False, yabloc_enabled=True) # pose_initializer

    # gyro_odometer: twist_estimator
    if 'gyro_odom' == twist_source:
        twist_group_actions += [launch_gyro_odometer()] # gyro_odometer

    # eagleye: Could be used as sources for twist, pose, and both
    if 'gnss' == pose_source and 'gnss' == twist_source:
        pose_twist_group_actions += [launch_eagleye(mode='pose_twist_estimator')] # eagleye (pose + twist mode)
        util_group_actions += launch_pose_initializer(system_run_mode, ndt_enabled=False, yabloc_enabled=False) # pose_initializer
    elif 'gnss' == pose_source:
        pose_group_actions += [launch_eagleye(mode='pose_estimator')] # eagleye (pose mode)
        util_group_actions += launch_pose_initializer(system_run_mode, ndt_enabled=False, yabloc_enabled=False) # pose_initializer
    elif 'gnss' == twist_source:
        twist_group_actions += [launch_eagleye(mode='twist_estimator')] # eagleye (twist mode)

    return [
        GroupAction(pose_group_actions),
        GroupAction(twist_group_actions),
        GroupAction(pose_twist_group_actions),
        GroupAction(util_group_actions),
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("pose_source", "lidar", "TO BE FILLED")
    add_launch_arg("twist_source", "gyro_odom", "TO BE FILLED")

    add_launch_arg("input/pointcloud", "", "input topic name")
    add_launch_arg("output/pointcloud", "downsample/pointcloud", "final output topic name")
    add_launch_arg("use_intra_process", "true", "use ROS 2 component container communication")

    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )
