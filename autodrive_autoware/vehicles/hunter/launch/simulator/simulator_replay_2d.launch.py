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
import os
import yaml

def generate_launch_description():
    config = os.path.join(get_package_share_directory('autodrive_hunter'), 'config/simulator', 'perception_2d.yaml')
    bog_map_name = yaml.safe_load(open(config, 'r'))['map_server']['ros__parameters']['map']

    autodrive_incoming_bridge = Node(
            package='autodrive_hunter',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
    )

    autodrive_outgoing_bridge = Node(
            package='autodrive_hunter',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            emulate_tty=True,
            output='screen',
    )
    
    world_to_map_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_tf',
            arguments = ['0.000', '0.000', '0.000', # (X, Y, Z) translation
                         '0.000', '0.000', '0.000', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'world', 'map']
    )

    world_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments = ['-1.1', '0.15', '0.03', # (X, Y, Z) translation
                         '0.00', '0.00', '0.00', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'world', 'odom']
    )

    pointcloud_to_laserscan_node = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('~/input/pointcloud', '/autodrive/hunter_1/lidar'),
                ('~/output/laserscan', '/scan_out'),
                ('~/output/pointcloud', '/cloud_out'),
                ('~/output/ray', '/ray_out'),
                ('~/output/stixel', '/stixel_out')
            ],
            parameters=[{
                'target_frame': 'lidar',
                'transform_tolerance': 0.01, # Time tolerance for transform lookups (sec)
                'min_height': 0.525, # 0.625 - 0.1 (0.625 is z coordinate of LIDAR frame), Minimum height to sample in the point cloud in meters
                'max_height': 0.725, # 0.625 + 0.1 (0.625 is z coordinate of LIDAR frame), Maximum height to sample in the point cloud in meters
                'angle_min': -3.1417, # -PI, Minimum scan angle in radians
                'angle_max': 3.1417, # PI, Maximum scan angle in radians
                'angle_increment': 0.01745, # PI/180, Resolution of laser scan in radians per ray
                'scan_time': 0.1, # 1/10 sec (10 Hz), Scan rate in seconds
                'range_min': 0.1, # Minimum ranges to return in meters
                'range_max': 20.0, # Maximum ranges to return in meters
                'use_inf': True # If disabled, report infinite range (no obstacle) as (range_max + 1). Otherwise report infinite range as (inf)
            }]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        emulate_tty=True,
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('autodrive_hunter'), 'maps', bog_map_name + '.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
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
        parameters=["{}/config/simulator/planning.yaml".format(get_package_share_directory('autodrive_hunter')),
        ],
    )

    control_node = Node(
        package="autodrive_trajectory_follower",
        executable="autodrive_trajectory_follower_exe",
        name="autodrive_trajectory_follower",
        output='screen',
        emulate_tty=True,
        parameters=["{}/config/simulator/control.yaml".format(get_package_share_directory('autodrive_hunter')),
        ],
        remappings=[
            ("input/kinematics", "/autodrive/hunter_1/odom"),
            ("input/trajectory", "/planning/trajectory"),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('autodrive_hunter'), 'rviz/simulator', 'replay_2d.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(autodrive_incoming_bridge)
    ld.add_action(autodrive_outgoing_bridge)
    ld.add_action(world_to_map_tf)
    ld.add_action(world_to_odom_tf)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(perception_node)
    ld.add_action(planning_node)
    ld.add_action(control_node)
    ld.add_action(rviz_node)
    return ld