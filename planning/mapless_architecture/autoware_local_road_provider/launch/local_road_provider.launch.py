# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # local_road_provider executable
            Node(
                package="local_road_provider",
                executable="local_road_provider",
                name="local_road_provider",
                namespace="mission_planner",
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
