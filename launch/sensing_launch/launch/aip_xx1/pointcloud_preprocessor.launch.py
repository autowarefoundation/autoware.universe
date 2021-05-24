
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


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
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
        p = yaml.safe_load(f)
    return p


def launch_setup(context, *args, **kwargs):

    pkg = 'pointcloud_preprocessor'

    vehicle_info = get_vehicle_info(context)

    # set concat filter as a component
    concat_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
        name='concatenate_data',
        remappings=[('output', 'concatenated/pointcloud')],
        parameters=[{
            'input_topics': ['/sensing/lidar/top/outlier_filtered/pointcloud',
                             '/sensing/lidar/left/outlier_filtered/pointcloud',
                             '/sensing/lidar/right/outlier_filtered/pointcloud'],
            'output_frame': LaunchConfiguration('base_frame'),
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    passthrough_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PassThroughFilterComponent',
        name='passthrough_filter',
        remappings=[
            ('input', 'top/outlier_filtered/pointcloud'),
            ('output', 'concatenated/pointcloud'),
        ],
        parameters=[{
            'output_frame': LaunchConfiguration('base_frame'),
            'min_z': vehicle_info['min_height_offset'],
            'max_z': vehicle_info['max_height_offset'],
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter',
        remappings=[
            ('input', 'concatenated/pointcloud'),
            ('output', 'measurement_range_cropped/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': -50.0,
            'max_x': 100.0,
            'min_y': -50.0,
            'max_y': 50.0,
            'min_z': vehicle_info['min_height_offset'],
            'max_z': vehicle_info['max_height_offset'],
            'negative': False,
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    ground_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::RayGroundFilterComponent',
        name='ray_ground_filter',
        remappings=[
            ('input', 'measurement_range_cropped/pointcloud'),
            ('output', 'no_ground/pointcloud')
        ],
        parameters=[{
            'general_max_slope': 10.0,
            'local_max_slope': 10.0,
            'min_height_threshold': 0.2,
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=[
            cropbox_component,
            ground_component,
        ],
        output='screen',
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration('use_concat_filter')),
    )

    passthrough_loader = LoadComposableNodes(
        composable_node_descriptions=[passthrough_component],
        target_container=container,
        condition=UnlessCondition(LaunchConfiguration('use_concat_filter')),
    )

    return [container, concat_loader, passthrough_loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('use_concat_filter', 'use_concat_filter')
    add_launch_arg('vehicle_param_file')
    add_launch_arg('use_multithread', 'False')
    add_launch_arg('use_intra_process', 'False')

    set_container_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_multithread'))
    )

    set_container_mt_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_multithread'))
    )

    return launch.LaunchDescription(launch_arguments +
                                    [set_container_executable,
                                     set_container_mt_executable] +
                                    [OpaqueFunction(function=launch_setup)])
