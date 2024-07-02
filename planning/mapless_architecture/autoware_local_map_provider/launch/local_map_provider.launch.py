# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # autoware_local_map_provider executable
            Node(
                package="autoware_local_map_provider",
                executable="autoware_local_map_provider",
                name="autoware_local_map_provider",
                namespace="mapless_architecture",
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
