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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    mpc_follower_param_path = \
        LaunchConfiguration('mpc_follower_param_path').perform(context)
    with open(mpc_follower_param_path, 'r') as f:
        mpc_follower_param = yaml.safe_load(f)['/**']['ros__parameters']
    pure_pursuit_param_path =  \
        LaunchConfiguration('pure_pursuit_param_path').perform(context)
    with open(pure_pursuit_param_path, 'r') as f:
        pure_pursuit_param = yaml.safe_load(f)['/**']['ros__parameters']
    velocity_controller_param_path = \
        LaunchConfiguration('velocity_controller_param_path').perform(context)
    with open(velocity_controller_param_path, 'r') as f:
        velocity_controller_param = yaml.safe_load(f)['/**']['ros__parameters']
    vehicle_cmd_gate_param_path = \
        LaunchConfiguration('vehicle_cmd_gate_param_path').perform(context)
    with open(vehicle_cmd_gate_param_path, 'r') as f:
        vehicle_cmd_gate_param = yaml.safe_load(f)['/**']['ros__parameters']
    # mpc follower
    mpc_follower_component = ComposableNode(
        package='mpc_follower',
        plugin='MPCFollower',
        name='mpc_follower',
        namespace='trajectory_follower',
        remappings=[
            ('~/input/reference_trajectory', '/planning/scenario_planning/trajectory'),
            ('~/input/current_velocity', '/localization/twist'),
            ('~/input/current_steering', '/vehicle/status/steering'),
            ('~/output/control_raw', 'lateral/control_cmd'),
            ('~/output/predicted_trajectory', 'predicted_trajectory'),
        ],
        parameters=[
            mpc_follower_param,
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )
    # pure pursuit
    pure_pursuit_component = ComposableNode(
        package='pure_pursuit',
        plugin='PurePursuitNode',
        name='pure_pursuit',
        namespace='trajectory_follower',
        remappings=[
            ('input/reference_trajectory', '/planning/scenario_planning/trajectory'),
            ('input/current_velocity', '/localization/twist'),
            ('output/control_raw', 'lateral/control_cmd'),
        ],
        parameters=[
            pure_pursuit_param,
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # velocity controller
    velocity_controller_component = ComposableNode(
        package='velocity_controller',
        plugin='VelocityController',
        name='velocity_controller',
        namespace='trajectory_follower',
        remappings=[
            ('~/current_velocity', '/localization/twist'),
            ('~/control_cmd', 'longitudinal/control_cmd'),
            ('~/current_trajectory', '/planning/scenario_planning/trajectory'),
        ],
        parameters=[
            velocity_controller_param,
            {
                'control_rate': LaunchConfiguration('control_rate'),
                'show_debug_info': LaunchConfiguration('show_debug_info'),
                'enable_smooth_stop': LaunchConfiguration('enable_smooth_stop'),
                'enable_pub_debug': LaunchConfiguration('enable_pub_debug'),
            }
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # latlon muxer
    latlon_muxer_component = ComposableNode(
        package='latlon_muxer',
        plugin='LatLonMuxer',
        name='latlon_muxer',
        namespace='trajectory_follower',
        remappings=[
            ('input/lateral/control_cmd', 'lateral/control_cmd'),
            ('input/longitudinal/control_cmd', 'longitudinal/control_cmd'),
            ('output/control_cmd', 'control_cmd'),
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # lane departure checker
    lane_departure_component = ComposableNode(
        package='lane_departure_checker',
        plugin='lane_departure_checker::LaneDepartureCheckerNode',
        name='lane_departure_checker_node',
        namespace='trajectory_follower',
        remappings=[
            ('~/input/twist', '/localization/twist'),
            ('~/input/lanelet_map_bin', '/map/vector_map'),
            ('~/input/route', '/planning/mission_planning/route'),
            ('~/input/reference_trajectory', '/planning/scenario_planning/trajectory'),
            ('~/input/predicted_trajectory', '/control/trajectory_follower/predicted_trajectory'),
        ],
        parameters=[
            [
                FindPackageShare('lane_departure_checker'),
                '/config/lane_departure_checker.param.yaml'
            ]
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # shift decider
    shift_decider_component = ComposableNode(
        package='shift_decider',
        plugin='ShiftDecider',
        name='shift_decider',
        remappings=[
            ('input/control_cmd', '/control/trajectory_follower/control_cmd'),
            ('output/shift_cmd', '/control/shift_decider/shift_cmd'),
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # vehicle cmd gate
    vehicle_cmd_gate_component = ComposableNode(
        package='vehicle_cmd_gate',
        plugin='VehicleCmdGate',
        name='vehicle_cmd_gate',
        remappings=[
            ('input/engage', '/autoware/engage'),
            ('input/system_emergency', '/system/emergency/is_emergency'),
            ('input/external_emergency_stop', '/remote/emergency_stop'),
            ('input/emergency', '/system/emergency/is_emergency'),
            ('input/gate_mode', '/remote/gate_mode_cmd'),
            ('input/steering', '/vehicle/status/steering'),

            ('input/auto/control_cmd', 'trajectory_follower/control_cmd'),
            ('input/auto/turn_signal_cmd', '/planning/turn_signal_decider/turn_signal_cmd'),
            ('input/auto/shift_cmd', '/control/shift_decider/shift_cmd'),

            ('input/remote/control_cmd', '/external/external_cmd_selector/control_cmd'),
            ('input/remote/turn_signal_cmd', '/external/external_cmd_selector/turn_signal_cmd'),
            ('input/remote/shift_cmd', '/external/external_cmd_selector/shift_cmd'),

            ('input/emergency/control_cmd', '/system/emergency/control_cmd'),
            ('input/emergency/turn_signal_cmd', '/system/emergency/turn_signal_cmd'),
            ('input/emergency/shift_cmd', '/system/emergency/shift_cmd'),

            ('output/vehicle_cmd', 'vehicle_cmd'),
            ('output/control_cmd', '/control/control_cmd'),
            ('output/shift_cmd', '/control/shift_cmd'),
            ('output/turn_signal_cmd', '/control/turn_signal_cmd'),
            ('output/gate_mode', '/control/current_gate_mode'),

            ('~/service/external_emergency_stop', '~/external_emergency_stop'),
            ('~/service/clear_external_emergency_stop', '~/clear_external_emergency_stop'),
        ],
        parameters=[
            vehicle_cmd_gate_param,
            {
                'use_emergency_handling': LaunchConfiguration('use_emergency_handling'),
                'use_external_emergency_stop': LaunchConfiguration('use_external_emergency_stop'),
            }
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # external cmd selector
    external_cmd_selector_component = ComposableNode(
        package='external_cmd_selector',
        plugin='ExternalCmdSelector',
        name='external_cmd_selector',
        remappings=[
            ('~/input/local/control_cmd', '/external/local/control_cmd'),
            ('~/input/local/shift_cmd', '/external/local/shift_cmd'),
            ('~/input/local/turn_signal_cmd', '/external/local/turn_signal_cmd'),
            ('~/input/remote/control_cmd', '/external/remote/control_cmd'),
            ('~/input/remote/shift_cmd', '/external/remote/shift_cmd'),
            ('~/input/remote/turn_signal_cmd', '/external/remote/turn_signal_cmd'),
            ('~/output/current_selector_mode', '~/current_selector_mode'),
            ('~/output/external_control_cmd',
             '/external/external_cmd_selector/external_control_cmd'),
            ('~/output/shift_cmd', '/external/external_cmd_selector/shift_cmd'),
            ('~/output/turn_signal_cmd', '/external/external_cmd_selector/turn_signal_cmd'),
            ('~/service/select_external_command', '~/select_external_command'),
        ],
        parameters=[
            {
                'initial_selector_mode': LaunchConfiguration('initial_selector_mode'),
            }
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # remote cmd converter
    remote_cmd_converter_component = ComposableNode(
        package='remote_cmd_converter',
        plugin='RemoteCmdConverter',
        name='remote_cmd_converter',
        remappings=[
            ('in/external_control_cmd', '/external/external_cmd_selector/external_control_cmd'),
            ('in/shift_cmd', '/external/external_cmd_selector/shift_cmd'),
            ('in/emergency_stop', '/remote/emergency_stop'),
            ('in/current_gate_mode', '/control/current_gate_mode'),
            ('in/twist', '/localization/twist'),
            ('out/control_cmd', '/external/external_cmd_selector/control_cmd'),
            ('out/latest_remote_control_cmd', '/remote/latest_remote_control_cmd'),
        ],
        parameters=[
            {
                'csv_path_accel_map': LaunchConfiguration('csv_path_accel_map'),
                'csv_path_brake_map': LaunchConfiguration('csv_path_brake_map'),

                'ref_vel_gain': LaunchConfiguration('ref_vel_gain'),
                'wait_for_first_topic': LaunchConfiguration('wait_for_first_topic'),
                'control_command_timeout': LaunchConfiguration('control_command_timeout'),
                'emergency_stop_timeout': LaunchConfiguration('emergency_stop_timeout'),
            }
        ],
        extra_arguments=[{
            'use_intra_process_comms': LaunchConfiguration('use_intra_process')
        }],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='control_container',
        namespace='',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=[
            velocity_controller_component,
            latlon_muxer_component,
            lane_departure_component,
            shift_decider_component,
            vehicle_cmd_gate_component,
            remote_cmd_converter_component,
            external_cmd_selector_component,
        ],
    )

    mpc_follower_loader = LoadComposableNodes(
        composable_node_descriptions=[mpc_follower_component],
        target_container=container,
        condition=LaunchConfigurationEquals(
            'lateral_control_mode', 'mpc_follower'
        ),
    )

    pure_pursuit_loader = LoadComposableNodes(
        composable_node_descriptions=[pure_pursuit_component],
        target_container=container,
        condition=LaunchConfigurationEquals(
            'lateral_control_mode', 'pure_pursuit'
        ),
    )

    group = GroupAction([
        PushRosNamespace('control'),
        container,
        mpc_follower_loader,
        pure_pursuit_loader
    ])

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(DeclareLaunchArgument(
            name, default_value=default_value, description=description))
    add_launch_arg('lateral_control_mode', 'mpc_follower',
                   'lateral control mode: `mpc_follower` or `pure_pursuit`')
    add_launch_arg(
        'mpc_follower_param_path',
        [
            FindPackageShare('control_launch'),
            '/config/mpc_follower/mpc_follower.param.yaml'
        ],
        'path to the parameter file of mpc_follower'
    )
    add_launch_arg(
        'pure_pursuit_param_path',
        [
            FindPackageShare('control_launch'),
            '/config/pure_pursuit/pure_pursuit.param.yaml'
        ],
        'path to the parameter file of pure_pursuit'
    )
    add_launch_arg(
        'velocity_controller_param_path',
        [
            FindPackageShare('control_launch'),
            '/config/velocity_controller/velocity_controller.param.yaml'
        ],
        'path to the parameter file of velocity controller'
    )
    add_launch_arg(
        'vehicle_cmd_gate_param_path',
        [
            FindPackageShare('control_launch'),
            '/config/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml'
        ],
        'path to the parameter file of vehicle_cmd_gate'
    )

    # velocity controller
    add_launch_arg('control_rate', '30.0', 'control rate')
    add_launch_arg('show_debug_info', 'false', 'show debug information')
    add_launch_arg('enable_smooth_stop', 'true',
                   'enable smooth stop (in velocity controller state)')
    add_launch_arg('enable_pub_debug', 'true', 'enable to publish debug information')

    # vehicle cmd gate
    add_launch_arg('use_emergency_handling', 'false', 'use emergency handling')
    add_launch_arg('use_external_emergency_stop', 'true', 'use external emergency stop')

    # external cmd selector
    add_launch_arg('initial_selector_mode', '1', '0: Local, 1: Remote')

    # remote cmd converter
    add_launch_arg(
        'csv_path_accel_map',
        [
            FindPackageShare('raw_vehicle_cmd_converter'),
            '/data/default/accel_map.csv'
        ],
        'csv file path for accel map'
    )
    add_launch_arg(
        'csv_path_brake_map',
        [
            FindPackageShare('raw_vehicle_cmd_converter'),
            '/data/default/brake_map.csv'
        ],
        'csv file path for brake map'
    )
    add_launch_arg('ref_vel_gain', '3.0', 'gain for remote command accel')
    add_launch_arg('wait_for_first_topic', 'true',
                   'disable topic disruption detection until subscribing first topics')
    add_launch_arg('control_command_timeout', '1.0', 'remote control command timeout')
    add_launch_arg('emergency_stop_timeout', '3.0', 'emergency stop timeout for remote heartbeat')

    add_launch_arg('use_intra_process', 'false', 'use ROS2 component container communication')
    add_launch_arg('use_multithread', 'false', 'use multithread')
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
    return launch.LaunchDescription(
        launch_arguments +
        [
            set_container_executable,
            set_container_mt_executable,
        ] +
        [
            OpaqueFunction(function=launch_setup)
        ]
    )
