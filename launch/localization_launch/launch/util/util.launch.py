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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable


def generate_launch_description():
    crop_box_component = ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_measurement_range',
        remappings=[
            ('input', LaunchConfiguration('input_sensor_points_topic')),
            ('output',
             'mesurement_range/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': -60.0,
            'max_x': 60.0,
            'min_y': -60.0,
            'max_y': 60.0,
            'min_z': -30.0,
            'max_z': 50.0,
            'negative': False,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )
    voxel_grid_downsample_component = ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
        name='voxel_grid_downsample_filter',
        remappings=[
            ('input',
             'mesurement_range/pointcloud'),
            ('output',
             LaunchConfiguration('output_voxel_grid_downsample_sensor_points_topic')),
        ],
        parameters=[{
            'voxel_size_x': 3.0,
            'voxel_size_y': 3.0,
            'voxel_size_z': 3.0,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )
    random_downsample_component = ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
        name='random_downsample_filter',
        remappings=[
            ('input',
             LaunchConfiguration('output_voxel_grid_downsample_sensor_points_topic')),
            ('output',
             LaunchConfiguration('output_downsample_sensor_points_topic')),
        ],
        parameters=[{
            'sample_num': 1500,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    composable_nodes = [crop_box_component,
                        voxel_grid_downsample_component,
                        random_downsample_component]

    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )
    return launch.LaunchDescription([load_composable_nodes])
