# MIT License

# Copyright (c) 2023 Tinker Twins, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    DeclareLaunchArgument('default_rviz', default_value='True')

    config = os.path.join(get_package_share_directory('autodrive_f1tenth'), 'config/gym_rviz', 'gym_rviz.yaml')
    has_opp = yaml.safe_load(open(config, 'r'))['gym_bridge']['ros__parameters']['num_agent'] > 1
    map_name = yaml.safe_load(open(config, 'r'))['gym_bridge']['ros__parameters']['map_name']

    gym_bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='gym_bridge',
        parameters=[config],
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('autodrive_f1tenth'), 'maps', map_name + '.yaml')},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    ego_vehicle_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'config', 'ego_racecar.xacro')])}],
        remappings=[
            ('/robot_description', 'ego_robot_description'),
        ]
    )

    opp_vehicle_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'config', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('autodrive_f1tenth'), 'rviz/gym_rviz', 'gym_rviz.rviz')]
    )

    ld = LaunchDescription()
    if LaunchConfiguration('default_rviz'):
        ld.add_action(rviz_node)
    ld.add_action(gym_bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_vehicle_publisher)
    if has_opp:
        ld.add_action(opp_vehicle_publisher)
    return ld