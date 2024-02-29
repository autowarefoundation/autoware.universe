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

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='autodrive_hunter',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='autodrive_hunter',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
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
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_tf',
            arguments = ['-6.27', '1.0', '0.17', # (X, Y, Z) translation
                         '0.00', '0.01', '0.00', # (Yaw, Pitch, Roll) body-fixed axis rotation
                         'world', 'odom']
        ),
    ])