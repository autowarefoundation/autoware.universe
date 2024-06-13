# Copyright 2024 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # hmi executable
            Node(
                package="hmi",
                executable="hmi",
                name="hmi",
                namespace="mission_planner",
                remappings=[
                    ("hmi_node/output/mission", "hmi_node/output/mission"),
                ],
                parameters=[],
                output="screen",
            ),
        ]
    )
