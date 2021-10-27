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

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode

import os
import yaml


def generate_launch_description():
    freespace_planner_param_path = os.path.join(
        get_package_share_directory('planning_launch'),
        'config',
        'scenario_planning',
        'parking',
        'freespace_planner',
        'freespace_planner.param.yaml',
    )
    with open(freespace_planner_param_path, 'r') as f:
        freespace_planner_param = yaml.safe_load(f)['/**']['ros__parameters']

    container = ComposableNodeContainer(
        name='parking_container',
        namespace='',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=[
            ComposableNode(
                package='costmap_generator',
                plugin='CostmapGenerator',
                name='costmap_generator',
                remappings=[
                    ('~/input/objects', '/perception/object_recognition/objects'),
                    ('~/input/points_no_ground', '/sensing/lidar/no_ground/pointcloud'),
                    ('~/input/vector_map', '/map/vector_map'),
                    ('~/input/scenario', '/planning/scenario_planning/scenario'),
                    ('~/output/grid_map', 'costmap_generator/grid_map'),
                    ('~/output/occupancy_grid', 'costmap_generator/occupancy_grid'),
                ],
                parameters=[
                    {
                        'costmap_frame': 'map',
                        'vehicle_frame': 'base_link',
                        'map_frame': 'map',
                        'update_rate': 10.0,
                        'use_wayarea': True,
                        'use_objects': True,
                        'use_points': True,
                        'grid_min_value': 0.0,
                        'grid_max_value': 1.0,
                        'grid_resolution': 0.2,
                        'grid_length_x': 70.0,
                        'grid_length_y': 70.0,
                        'grid_position_x': 0.0,
                        'grid_position_y': 0.0,
                        'maximum_lidar_height_thres': 0.3,
                        'minimum_lidar_height_thres': -2.2,
                        'expand_polygon_size': 1.0,
                        'size_of_expansion_kernel': 9,
                    },
                ],
                extra_arguments=[{
                    'use_intra_process_comms': LaunchConfiguration('use_intra_process')
                }],
            ),
            ComposableNode(
                package='freespace_planner',
                plugin='freespace_planner::FreespacePlannerNode',
                name='freespace_planner',
                remappings=[
                    ('~/input/route', '/planning/mission_planning/route'),
                    ('~/input/occupancy_grid', 'costmap_generator/occupancy_grid'),
                    ('~/input/scenario', '/planning/scenario_planning/scenario'),
                    ('~/input/twist', '/localization/twist'),
                    ('~/output/trajectory',
                     '/planning/scenario_planning/parking/trajectory'),
                    ('is_completed', '/planning/scenario_planning/parking/is_completed'),
                ],
                parameters=[
                    freespace_planner_param,
                ],
                extra_arguments=[{
                    'use_intra_process_comms': LaunchConfiguration('use_intra_process')
                }],
            )
        ],
    )

    set_container_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_multithread')),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        'container_executable',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_multithread')),
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_intra_process', default_value='false',
                              description='use ROS2 component container communication'),
        DeclareLaunchArgument('use_multithread', default_value='false',
                              description='use multithread'),
        set_container_executable,
        set_container_mt_executable,
        GroupAction([
            PushRosNamespace('parking'),
            container
        ])
    ])
