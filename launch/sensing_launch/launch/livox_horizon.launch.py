
# Copyright 2020 Tier IV, Inc. All rights reserved.
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
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(context):
    path = LaunchConfiguration('vehicle_param_file').perform(context)
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    p['vehicle_length'] = p['front_overhang'] + p['wheel_base'] + p['rear_overhang']
    p['vehicle_width'] = p['wheel_tread'] + p['left_overhang'] + p['right_overhang']
    p['min_longitudinal_offset'] = -p['rear_overhang']
    p['max_longitudinal_offset'] = p['front_overhang'] + p['wheel_base']
    p['min_lateral_offset'] = -(p['wheel_tread'] / 2.0 + p['right_overhang'])
    p['max_lateral_offset'] = p['wheel_tread'] / 2.0 + p['left_overhang']
    p['min_height_offset'] = 0.0
    p['max_height_offset'] = p['vehicle_height']
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration('vehicle_mirror_param_file').perform(context)
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return p


def launch_setup(context, *args, **kwargs):

    pkg = 'pointcloud_preprocessor'

    vehicle_info = get_vehicle_info(context)
    vehicle_mirror_info = get_vehicle_mirror_info(context)

    bd_code_param_path = LaunchConfiguration('bd_code_param_path').perform(context)
    with open(bd_code_param_path, 'r') as f:
        bd_code_param = yaml.safe_load(f)['/**']['ros__parameters']

    # livox driver
    livox_driver_component = ComposableNode(
        package='livox_ros2_driver',
        plugin='livox_ros::LivoxDriver',
        name='livox_driver',
        parameters=[
            {
                'xfe_format': LaunchConfiguration('xfe_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': LaunchConfiguration('data_src'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': LaunchConfiguration('output_type'),
                'lvx_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path': LaunchConfiguration('user_config_path'),
                'frame_id': LaunchConfiguration('sensor_frame'),
            },
            bd_code_param,
        ]
    )

    # set self crop box filter as a component
    cropbox_self_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='self_crop_box_filter',
        remappings=[
            ('input', 'livox/lidar'),
            ('output', 'self_cropped/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': vehicle_info['min_longitudinal_offset'],
            'max_x': vehicle_info['max_longitudinal_offset'],
            'min_y': vehicle_info['min_lateral_offset'],
            'max_y': vehicle_info['max_lateral_offset'],
            'min_z': vehicle_info['min_height_offset'],
            'max_z': vehicle_info['max_height_offset'],
            'negative': True,
        }]
    )

    # set mirror crop box filter as a component
    cropbox_mirror_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='mirror_crop_box_filter',
        remappings=[
            ('input', 'self_cropped/pointcloud'),
            ('output', 'mirror_cropped/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': vehicle_mirror_info['min_longitudinal_offset'],
            'max_x': vehicle_mirror_info['max_longitudinal_offset'],
            'min_y': vehicle_mirror_info['min_lateral_offset'],
            'max_y': vehicle_mirror_info['max_lateral_offset'],
            'min_z': vehicle_mirror_info['min_height_offset'],
            'max_z': vehicle_mirror_info['max_height_offset'],
            'negative': True,
        }]
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            cropbox_self_component,
            cropbox_mirror_component,
        ],
        output='screen',
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[livox_driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_driver')),
    )

    return [container, loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('xfe_format', '0')
    add_launch_arg('multi_topic', '0')
    add_launch_arg('data_src', '0')
    add_launch_arg('publish_freq', '10.0')
    add_launch_arg('output_type', '0')
    add_launch_arg('lvx_file_path', 'livox_test.lvx')
    add_launch_arg('user_config_path', os.path.join(get_package_share_directory(
        'livox_ros2_driver'), 'config/livox_lidar_config.json'))
    add_launch_arg('bd_code_param_path')
    add_launch_arg('launch_driver')
    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('sensor_frame', 'livox_frame')
    add_launch_arg('vehicle_param_file')
    add_launch_arg('vehicle_mirror_param_file')

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
