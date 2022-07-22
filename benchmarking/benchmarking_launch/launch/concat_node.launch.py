from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
                name="concatenate_data",
                remappings=[("output", "/sensing/lidar/concatenated/pointcloud")],
                parameters=[
                    {
                        "input_topics": [
                            "/point_cloud/front_lidar",
                            "/point_cloud/rear_lidar",
                            "/point_cloud/side_left_lidar",
                            "/point_cloud/side_right_lidar",
                            "/point_cloud/top_lidar",
                        ],
                        "output_frame": "base_frame",
                    }
                ],
            ),
        ]
    )
