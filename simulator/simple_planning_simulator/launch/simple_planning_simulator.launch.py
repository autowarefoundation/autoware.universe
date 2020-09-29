from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os



context = LaunchContext()


def find_pack(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)

def generate_launch_description():

    simple_planning_simulator_param_file = os.path.join(find_pack('simple_planning_simulator'), 'config/simple_planning_simulator.param.yaml')

    print(simple_planning_simulator_param_file)

    simple_planning_simulator_param = DeclareLaunchArgument(
        'simple_planning_simulator_param_file',
        default_value=simple_planning_simulator_param_file,
        description='Path to config file for simple_planning_simulator'
    )

    simple_planning_simulator = Node(
        package='simple_planning_simulator',
        node_executable='simple_planning_simulator_exe',
        node_name='simple_planning_simulator',
        node_namespace='simulation',
        output='screen',
        parameters=[LaunchConfiguration('simple_planning_simulator_param_file'),
            {
                "/vehicle_info/wheel_base": 4.0,
                "random_seed": 1
            }
        ],
        remappings=[
            ('base_trajectory', '/planning/scenario_planning/trajectory'),
            ('output/current_pose', 'current_pose'),
            ('output/current_twist', '/vehicle/status/twist'),
            ('output/status', '/vehicle/status'),
            ('output/control_mode', '/vehicle/status/control_mode'),
            ('input/vehicle_cmd', '/control/vehicle_cmd'),
            ('input/turn_signal_cmd', '/control/turn_signal_cmd'),
            ('input/initial_twist', '/initialtwist'),
            ('input/initial_pose', '/initialpose'),
            ('input/engage', '/vehicle/engage'),
        ]
    )


    return LaunchDescription([
        simple_planning_simulator_param,
        simple_planning_simulator
    ])
