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
from launch.substitutions import LaunchConfiguration
import os
import yaml

def generate_launch_description():
    config = os.path.join(get_package_share_directory('autodrive_nigel'), 'config/testbed', 'perception.yaml')
    map_name = yaml.safe_load(open(config, 'r'))['map_server']['ros__parameters']['map']

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    lidar_bringup_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            emulate_tty=True,
            output='screen',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                           'scan_mode': scan_mode
                         }],
        remappings=[
            ("/scan", "/autodrive/nigel_1/lidar"),
        ],
    )
    
    odometry_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/autodrive/nigel_1/lidar',
                'odom_topic': '/autodrive/nigel_1/odom',
                'publish_tf': True,
                'base_frame_id': 'nigel_1',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq' : 20.0}],
    )

    map_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nigel_to_lidar_tf',
            arguments = ['0.0', '0.0', '0.0', # (X, Y, Z) translation
                         '0.0', '0.0', '0.0', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'map', 'odom']
    )
    
    nigel_to_lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nigel_to_lidar_tf',
            arguments = ['0.1445', '0.0', '0.1757', # (X, Y, Z) translation
                         '3.1416', '0.00', '0.00', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'nigel_1', 'lidar']
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        emulate_tty=True,
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('autodrive_nigel'), 'maps', map_name + '.yaml')},
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
        parameters=["{}/config/testbed/planning.yaml".format(get_package_share_directory('autodrive_nigel')),
        ],
    )

    control_node = Node(
        package="autodrive_trajectory_follower",
        executable="autodrive_trajectory_follower_exe",
        name="autodrive_trajectory_follower",
        output='screen',
        emulate_tty=True,
        parameters=["{}/config/testbed/control.yaml".format(get_package_share_directory('autodrive_nigel')),
        ],
        remappings=[
            ("input/kinematics", "/autodrive/nigel_1/odom"),
            ("input/trajectory", "/planning/trajectory"),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('autodrive_nigel'), 'rviz/testbed', 'record.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(lidar_bringup_node)
    ld.add_action(odometry_node)
    ld.add_action(map_to_odom_tf)
    ld.add_action(nigel_to_lidar_tf)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(perception_node)
    ld.add_action(planning_node)
    ld.add_action(control_node)
    ld.add_action(rviz_node)
    return ld