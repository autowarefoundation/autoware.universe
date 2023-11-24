import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('f1tenth_stack'), 'launch'),
            '/bringup_launch.py']),
        )

    control = Node(
        package="trajectory_follower_f1tenth",
        executable="f1tenth_trajectory_follower_exe",
        name="f1tenth_trajectory_follower",
        # output='screen',
        emulate_tty=True,
        parameters=[
            {"use_external_target_vel": True},
            {"external_target_vel": 5.0},
            {"lateral_deviation": 0.0}
        ],
        remappings=[
            ("input/kinematics", "/ego_racecar/odom"),
            ("input/trajectory", "/planning/trajectory"),
            ("output/control_cmd", "/vehicle/command/manual_control_cmd"),
        ]
    )

    recordreplay = Node(
        package="recordreplay_planner_nodes",
        executable="recordreplay_planner_node_exe",
        name="recordreplay_planner",
        namespace="planning",
        output="screen",
        parameters=["{}/config/testbed/planning.yaml".format(get_package_share_directory('autodrive_f1tenth')),
        ],
    )

    return LaunchDescription([
        sim,
        control,
        recordreplay
    ])
