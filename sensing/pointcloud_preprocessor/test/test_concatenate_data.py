#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
from launch.logging import get_logger
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

logger = get_logger(__name__)

INPUT_LIDAR_TOPICS = [
    "/test/sensing/lidar/front_upper/pointcloud",
    "/test/sensing/lidar/front_lower/pointcloud",
    "/test/sensing/lidar/left_upper/pointcloud",
    "/test/sensing/lidar/left_lower/pointcloud",
    "/test/sensing/lidar/right_upper/pointcloud",
    "/test/sensing/lidar/right_lower/pointcloud",
    "/test/sensing/lidar/rear_upper/pointcloud",
    "/test/sensing/lidar/rear_lower/pointcloud",
]
INPUT_LIDAR_TOPICS_OFFSET = [0.025, 0.025, 0.01, 0.0, 0.05, 0.05, 0.05, 0.05]
TIMEOUT_SEC = 0.075
INPUT_LIDAR_NUM_POINTS = [10, 10, 10, 10, 10, 10, 10, 10]


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_data",
            remappings=[
                ("~/input/twist", "/test/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "/test/sensing/lidar/concatenated/pointcloud"),
            ],
            parameters=[
                {
                    "input_topics": INPUT_LIDAR_TOPICS,
                    "input_offset": INPUT_LIDAR_TOPICS_OFFSET,
                    "timeout_sec": TIMEOUT_SEC,
                    "output_frame": "base_link",
                    "input_twist_topic_type": "twist",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    container = ComposableNodeContainer(
        name="test_concateante_data_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestConcatenateData(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.test_node = rclpy.create_node("test_node")
        self.evaluation_time = 2  # 200ms

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    @staticmethod
    def pointcloud2_to_xyz_array(cloud_msg):
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)
        cloud_arr = np.reshape(cloud_arr, (cloud_msg.width, int(cloud_msg.point_step / 4)))
        return cloud_arr[:, :3]

    @staticmethod
    def generate_pointcloud(num_points, header, timestamp_offset_per_point=0.01):
        points_xyz = np.random.rand(num_points, 3).astype(np.float32)
        timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + timestamp_offset_per_point * i
                for i in range(num_points)
            ]
        ).astype(np.float64)
        xyz_data = points_xyz.tobytes()
        timestamp_data = timestamps.tobytes()
        pointcloud_data = b"".join(
            xyz_data[i * 12 : i * 12 + 12] + timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(xyz_data))
        )
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
        ]
        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=10,
            is_dense=True,
            is_bigendian=False,
            point_step=20,  # 4 float32 fields * 4 bytes/field
            row_step=20 * num_points,  # point_step * width
            fields=fields,
            data=pointcloud_data,
        )
        return pointcloud_msg

    @staticmethod
    def offset_timestamp(nanosec: int, offset_ms: int) -> int:
        """
        Offset the nanosecond part of a timestamp.
        """
        # Convert nanoseconds to microseconds and round to the nearest hundred microseconds
        # Add the offset
        new_microsec = round(nanosec / 1000, -2) + offset_ms * 1000

        return int((new_microsec % 1000000) * 1000)

    def test_node_link(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publishers
        velocity_pub = self.test_node.create_publisher(
            TwistWithCovarianceStamped,
            "/test/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        input_lidar_topics = INPUT_LIDAR_TOPICS
        pointcloud_pub = {}
        for input_lidar_topic in input_lidar_topics:
            pointcloud_pub[input_lidar_topic] = self.test_node.create_publisher(
                PointCloud2,
                input_lidar_topic,
                qos_profile=sensor_qos,
            )

        # Subscribers
        received_data = {"msg": None}

        def pointcloud_callback(msg):
            received_data["msg"] = msg

        self.test_node.create_subscription(
            PointCloud2,
            "/test/sensing/lidar/concatenated/pointcloud",
            pointcloud_callback,
            qos_profile=sensor_qos,
        )

        # Wait for composable node launches
        rclpy.spin_once(self.test_node, timeout_sec=1.0)

        # Prepare header
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Publish velocity data
        for i in range(50):
            header_tmp = Header()
            header_tmp.stamp = self.test_node.get_clock().now().to_msg()
            header_tmp.frame_id = "base_link"

            velocity_msg = TwistWithCovarianceStamped()
            velocity_msg.header = header_tmp
            velocity_msg.twist.twist.linear.x = 1.0

            # imu_pub.publish(imu_msg)
            velocity_pub.publish(velocity_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Publish pointcloud data
        base_timestamp_nanosec: int = header.stamp.nanosec
        for i, input_lidar_topic in enumerate(input_lidar_topics):
            num_points = INPUT_LIDAR_NUM_POINTS[i]
            offset_ms = INPUT_LIDAR_TOPICS_OFFSET[i]
            header.stamp.nanosec = self.offset_timestamp(base_timestamp_nanosec, offset_ms)
            pointcloud_msg = self.generate_pointcloud(num_points, header)
            pointcloud_pub[input_lidar_topic].publish(pointcloud_msg)

        # Wait for output with a timeout
        start_time = self.test_node.get_clock().now()
        while self.test_node.get_clock().now() - start_time < rclpy.duration.Duration(
            seconds=self.evaluation_time
        ):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if received_data["msg"] is not None:
                break

        # Check if the output was received
        self.assertIsNotNone(received_data["msg"], "Did not receive output pointcloud data")

        # Check that the received pointcloud data has same length as all_num_points
        received_pointcloud = received_data["msg"]
        all_num_points = sum(INPUT_LIDAR_NUM_POINTS)
        self.assertEqual(
            received_pointcloud.width,
            all_num_points,
            "The received pointcloud data has a different length than expected",
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
