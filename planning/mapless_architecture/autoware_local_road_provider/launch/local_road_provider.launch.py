# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # autoware_local_road_provider executable
            Node(
                package="autoware_local_road_provider",
                executable="autoware_local_road_provider",
                name="autoware_local_road_provider",
                namespace="mapless_architecture",
                remappings=[
                    (
                        "local_road_provider_node/output/road_segments",
                        "local_road_provider_node/output/road_segments",
                    ),
                    ("local_road_provider_node/input/lanelets", "/env/output/driving_corridors"),
                ],
                parameters=[],
                output="screen",
            ),
        ]
    )
