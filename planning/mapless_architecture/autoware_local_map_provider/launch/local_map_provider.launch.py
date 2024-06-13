# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # local_map_provider executable
            Node(
                package="local_map_provider",
                executable="local_map_provider",
                name="local_map_provider",
                namespace="mission_planner",
                remappings=[
                    (
                        "local_map_provider_node/output/local_map",
                        "local_map_provider_node/output/local_map",
                    ),
                    (
                        "local_map_provider_node/input/road_segments",
                        "local_road_provider_node/output/road_segments",
                    ),
                ],
                parameters=[],
                output="screen",
            ),
        ]
    )
