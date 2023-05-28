import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('f1tenth_gym_ros'), 'launch'),
            '/gym_bridge_launch.py']),
        )

    control = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trajectory_follower_f1tenth'), 'launch'),
            '/trajectory_follower_f1tenth.launch.xml'])
        )

    recordreplay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('recordreplay_planner_nodes'), 'launch'),
            '/recordreplay_planner_node.launch.py']),
        )

    return LaunchDescription([
        sim,
        control,
        recordreplay
    ])