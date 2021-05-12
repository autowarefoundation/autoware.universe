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
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

import os
import yaml


def generate_launch_description():
    # obstacle avoidance planner
    obstacle_avoidance_planner_param_path = os.path.join(
        get_package_share_directory('planning_launch'),
        'config',
        'scenario_planning',
        'lane_driving',
        'motion_planning',
        'obstacle_avoidance_planner',
        'obstacle_avoidance_planner.param.yaml',
    )
    with open(obstacle_avoidance_planner_param_path, 'r') as f:
        obstacle_avoidance_planner_param = yaml.safe_load(f)['/**']['ros__parameters']
    obstacle_avoidance_planner_component = ComposableNode(
        package='obstacle_avoidance_planner',
        plugin='ObstacleAvoidancePlanner',
        name='obstacle_avoidance_planner',
        namespace='',
        remappings=[
            ('~/input/objects', '/perception/object_recognition/objects'),
            ('~/input/path', LaunchConfiguration('input_path_topic')),
            ('~/output/path', 'obstacle_avoidance_planner/trajectory'),
        ],
        parameters=[
            obstacle_avoidance_planner_param,
            {'is_showing_debug_info': False},
            {'is_stopping_if_outside_drivable_area': True},
        ],
        extra_arguments=[
            {'use_intra_process_comms': LaunchConfiguration('use_intra_process')}
        ],
    )

    # surround obstacle checker
    surround_obstacle_checker_param_path = os.path.join(
        get_package_share_directory('planning_launch'),
        'config',
        'scenario_planning',
        'lane_driving',
        'motion_planning',
        'surround_obstacle_checker',
        'surround_obstacle_checker.param.yaml',
    )
    with open(surround_obstacle_checker_param_path, 'r') as f:
        surround_obstacle_checker_param = yaml.safe_load(f)['/**']['ros__parameters']
    surround_obstacle_checker_component = ComposableNode(
        package='surround_obstacle_checker',
        plugin='SurroundObstacleCheckerNode',
        name='surround_obstacle_checker',
        namespace='',
        remappings=[
            ('~/output/no_start_reason',
             '/planning/scenario_planning/status/no_start_reason'),
            ('~/output/stop_reasons',
             '/planning/scenario_planning/status/stop_reasons'),
            ('~/output/trajectory', 'surround_obstacle_checker/trajectory'),
            ('~/input/pointcloud', '/sensing/lidar/no_ground/pointcloud'),
            ('~/input/objects', '/perception/object_recognition/objects'),
            ('~/input/twist', '/localization/twist'),
            ('~/input/trajectory', 'obstacle_avoidance_planner/trajectory'),
        ],
        parameters=[
            surround_obstacle_checker_param,
        ],
        extra_arguments=[
            {'use_intra_process_comms': LaunchConfiguration('use_intra_process')}
        ],
    )

    # relay
    relay_component = ComposableNode(
        package='topic_tools',
        plugin='topic_tools::RelayNode',
        name='skip_surround_obstacle_check_relay',
        namespace='',
        parameters=[{
            'input_topic': 'obstacle_avoidance_planner/trajectory',
            'output_topic': 'surround_obstacle_checker/trajectory',
            'type': 'autoware_planning_msgs/msg/Trajectory',
        }],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # obstacle stop planner
    obstacle_stop_planner_param_path = os.path.join(
        get_package_share_directory('planning_launch'),
        'config',
        'scenario_planning',
        'lane_driving',
        'motion_planning',
        'obstacle_stop_planner',
        'obstacle_stop_planner.param.yaml',
    )
    obstacle_stop_planner_acc_param_path = os.path.join(
        get_package_share_directory('planning_launch'),
        'config',
        'scenario_planning',
        'lane_driving',
        'motion_planning',
        'obstacle_stop_planner',
        'adaptive_cruise_control.param.yaml',
    )
    with open(obstacle_stop_planner_param_path, 'r') as f:
        obstacle_stop_planner_param = yaml.safe_load(f)['/**']['ros__parameters']
    with open(obstacle_stop_planner_acc_param_path, 'r') as f:
        obstacle_stop_planner_acc_param = yaml.safe_load(f)['/**']['ros__parameters']
    obstacle_stop_planner_component = ComposableNode(
        package='obstacle_stop_planner',
        plugin='motion_planning::ObstacleStopPlannerNode',
        name='obstacle_stop_planner',
        namespace='',
        remappings=[
            ('~/output/stop_reason',
             '/planning/scenario_planning/status/stop_reason'),
            ('~/output/stop_reasons',
             '/planning/scenario_planning/status/stop_reasons'),
            ('~/output/trajectory',
             '/planning/scenario_planning/lane_driving/trajectory'),
            ('~/input/pointcloud', '/sensing/lidar/no_ground/pointcloud'),
            ('~/input/objects', '/perception/object_recognition/objects'),
            ('~/input/twist', '/localization/twist'),
            ('~/input/trajectory', 'surround_obstacle_checker/trajectory'),
        ],
        parameters=[
            obstacle_stop_planner_param,
            obstacle_stop_planner_acc_param,
            {'enable_slow_down': False}
        ],
        extra_arguments=[
            {'use_intra_process_comms': LaunchConfiguration('use_intra_process')}
        ],
    )

    container = ComposableNodeContainer(
        name='motion_planning_container',
        namespace='',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=[
            obstacle_avoidance_planner_component,
            obstacle_stop_planner_component
        ],
    )

    surround_obstacle_checker_loader = LoadComposableNodes(
        composable_node_descriptions=[surround_obstacle_checker_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration('use_surround_obstacle_check')),
    )

    relay_loader = LoadComposableNodes(
        composable_node_descriptions=[relay_component],
        target_container=container,
        condition=UnlessCondition(LaunchConfiguration('use_surround_obstacle_check')),
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
        DeclareLaunchArgument(
            'input_path_topic',
            default_value='/planning/scenario_planning/lane_driving/behavior_planning/path'),
        DeclareLaunchArgument(
            'use_surround_obstacle_check',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'use_intra_process',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'use_multithread',
            default_value='false'
        ),
        set_container_executable,
        set_container_mt_executable,
        container,
        surround_obstacle_checker_loader,
        relay_loader,
    ])
