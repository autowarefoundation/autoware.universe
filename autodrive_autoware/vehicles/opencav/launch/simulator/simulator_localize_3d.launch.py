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
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(get_package_share_directory('autodrive_opencav'), 'config/simulator', 'perception_3d.yaml')
    pcd_map_name = yaml.safe_load(open(config, 'r'))['pcd_map_loader']['ros__parameters']['map']
    pcd_map_path = os.path.join(get_package_share_directory('autodrive_opencav'), 'maps', pcd_map_name + '.pcd')
    
    autodrive_incoming_bridge = Node(
            package='autodrive_opencav',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
    )

    autodrive_outgoing_bridge = Node(
            package='autodrive_opencav',
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

    pointcloud_map_loader = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory("map_loader"),
                "launch/pointcloud_map_loader.launch.xml",
            )
        ),
        launch_arguments = {
            'pointcloud_map_path' : pcd_map_path,
            }.items(),
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('autodrive_opencav'), 'rviz/simulator', 'localize_3d.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(autodrive_incoming_bridge)
    ld.add_action(autodrive_outgoing_bridge)
    ld.add_action(world_to_map_tf)
    ld.add_action(world_to_odom_tf)
    ld.add_action(pointcloud_map_loader)
    ld.add_action(rviz_node)
    return ld