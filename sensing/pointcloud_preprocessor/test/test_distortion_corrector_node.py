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

import copy
import time
import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

# TODO: remove unused later
import launch
import launch.actions
from launch.logging import get_logger
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

logger = get_logger(__name__)

number_of_points = 10
points_time_offset = 10  # 10 ms


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node_unused_imu",
            parameters=[
                {"use_imu": False},
            ],
            remappings=[
                ("~/input/twist", "/test/twist_with_covariance"),
                ("~/input/imu", "/test/imu/imu_data"),
                ("~/input/pointcloud", "/test/lidar/top/pointcloud_raw"),
                ("~/output/pointcloud", "/test/lidar/top/imu_unused/pointcloud_rectified"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node_used_imu",
            parameters=[
                {"use_imu": True},
            ],
            remappings=[
                ("~/input/twist", "/test/twist_with_covariance"),
                ("~/input/imu", "/test/imu/imu_data"),
                ("~/input/pointcloud", "/test/lidar/top/pointcloud_raw"),
                ("~/output/pointcloud", "/test/lidar/top/imu_used/pointcloud_rectified"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    container = ComposableNodeContainer(
        name="test_distortion_corrector_container",
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


def create_pointcloud_header(is_lidar_frame: bool, pointcloud_stamp):
    header = Header()
    header.stamp = copy.deepcopy(pointcloud_stamp)
    if is_lidar_frame:
        header.frame_id = "lidar_top"
    else:
        header.frame_id = "base_link"

    return copy.deepcopy(header)


def create_points(flag: bool):
    if flag:
        list_points = [
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 10.0],
            [20.0, 0.0, 0.0],
            [0.0, 20.0, 0.0],
            [0.0, 0.0, 20.0],
            [30.0, 0.0, 0.0],
            [0.0, 30.0, 0.0],
            [0.0, 0.0, 30.0],
            [10.0, 10.0, 10.0],
        ]
        assert len(list_points) == number_of_points
    else:
        list_points = []

    np_points = np.array(list_points, dtype=np.float32).reshape(-1, 3)
    points = np_points.tobytes()
    return points


def create_point_stamps(is_generate_points: bool, pointcloud_stamp):
    point_stamp = copy.deepcopy(pointcloud_stamp)
    list_timestamps = []
    if is_generate_points:
        for i in range(number_of_points):
            if i == 0:
                # For the first point, do not add the time offset
                list_timestamps.append(point_stamp.sec + point_stamp.nanosec * 1e-9)
            else:
                point_stamp = add_ms_to_stamp(point_stamp, points_time_offset)
                # timestamps.append(point_stamp.sec + point_stamp.nanosec * 1e-9)
                list_timestamps.append(point_stamp.sec + point_stamp.nanosec * 1e-9)

    np_timestamps = np.array(list_timestamps, dtype=np.float64)
    timestamps = np_timestamps.tobytes()
    return timestamps


def get_pointcloud_msg(is_generate_points: bool, is_lidar_frame: bool, pointcloud_stamp):
    pointcloud_header = create_pointcloud_header(is_lidar_frame, pointcloud_stamp)
    points = create_points(is_generate_points)
    timestamps = create_point_stamps(is_generate_points, pointcloud_stamp)

    pointcloud_data = b"".join(
        points[i * 12 : i * 12 + 12] + timestamps[i * 8 : i * 8 + 8] for i in range(len(points))
    )
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
    ]
    pointcloud_msg = PointCloud2(
        header=pointcloud_header,
        height=1,
        width=number_of_points,
        is_dense=True,
        is_bigendian=False,
        point_step=20,  # 3 float32 fields * 4 bytes/field + 1 float64 field * 8 bytes/field
        row_step=20 * number_of_points,  # point_step * width
        fields=fields,
        data=pointcloud_data,
    )

    data_array = np.frombuffer(pointcloud_msg.data, dtype=np.float32)
    for i in range(number_of_points):
        x = data_array[i * 5]
        y = data_array[i * 5 + 1]
        z = data_array[i * 5 + 2]
        time_stamp = np.frombuffer(
            pointcloud_msg.data[i * 20 + 12 : i * 20 + 20], dtype=np.float64
        )[0]
        print(f"Point {i}: x={x}, y={y}, z={z}, time_stamp={time_stamp}")

    return pointcloud_msg


def generate_static_transform_msg():
    tf_top_lidar_msg = TransformStamped()
    tf_top_lidar_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    tf_top_lidar_msg.header.frame_id = "base_link"
    tf_top_lidar_msg.child_frame_id = "top_lidar"
    tf_top_lidar_msg.transform.translation.x = 0.0
    tf_top_lidar_msg.transform.translation.y = 0.0
    tf_top_lidar_msg.transform.translation.z = 5.0
    tf_top_lidar_msg.transform.rotation.x = 0.0
    tf_top_lidar_msg.transform.rotation.y = 0.0
    tf_top_lidar_msg.transform.rotation.z = 0.0
    tf_top_lidar_msg.transform.rotation.w = 1.0

    tf_imu_msg = TransformStamped()
    tf_imu_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    tf_imu_msg.header.frame_id = "base_link"
    tf_imu_msg.child_frame_id = "imu_link"
    tf_imu_msg.transform.translation.x = 0.0
    tf_imu_msg.transform.translation.y = 5.0
    tf_imu_msg.transform.translation.z = 0.0
    tf_imu_msg.transform.rotation.x = 0.0
    tf_imu_msg.transform.rotation.y = 0.0
    tf_imu_msg.transform.rotation.z = 0.0
    tf_imu_msg.transform.rotation.w = 1.0

    return [tf_top_lidar_msg, tf_imu_msg]


def add_ms_to_stamp(stamp: Time, ms: int) -> Time:
    stamp.nanosec = int(stamp.nanosec + ms * 1000000)  # Convert ms to nanoseconds and add
    if stamp.nanosec >= 1e9:
        stamp.sec += 1
        stamp.nanosec -= int(1e9)
    return stamp


def subtract_ms_from_stamp(stamp: Time, ms: int) -> Time:
    stamp.nanosec = int(stamp.nanosec - ms * 1000000)  # Convert ms to nanoseconds and subtract
    if stamp.nanosec < 0:
        stamp.sec -= 1
        stamp.nanosec += int(1e9)
    return stamp


def generate_twist_msgs(pointcloud_stamp):
    twist_msgs = []
    twist_stamp = copy.deepcopy(pointcloud_stamp)
    twist_stamp = subtract_ms_from_stamp(twist_stamp, 5)

    for i in range(6):
        twist_header = Header()
        twist_header.stamp = twist_stamp
        twist_header.frame_id = "base_link"
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header = copy.deepcopy(twist_header)
        twist_msg.twist.twist.linear.x = 10.0 + i * 2
        twist_msg.twist.twist.linear.y = 0.0
        twist_msg.twist.twist.linear.z = 0.0
        twist_msg.twist.twist.angular.x = 0.0
        twist_msg.twist.twist.angular.y = 0.0
        twist_msg.twist.twist.angular.z = 0.02 + i * 0.01

        twist_msgs.append(twist_msg)
        twist_stamp = add_ms_to_stamp(twist_stamp, 24)

    return twist_msgs


# generate imu msg
def generate_imu_msgs(pointcloud_stamp):
    imu_msgs = []
    imu_stamp = copy.deepcopy(pointcloud_stamp)
    imu_stamp = subtract_ms_from_stamp(imu_stamp, 10)

    for i in range(6):
        imu_header = Header()
        imu_header.stamp = imu_stamp
        imu_header.frame_id = "imu_link"
        imu_msg = Imu()
        imu_msg.header = copy.deepcopy(imu_header)
        imu_msg.angular_velocity.x = 0.01 + i * 0.005
        imu_msg.angular_velocity.y = -0.02 + i * 0.005
        imu_msg.angular_velocity.z = 0.05 + i * 0.005

        imu_msgs.append(imu_msg)
        imu_stamp = add_ms_to_stamp(imu_stamp, 29)

    return imu_msgs


def pointcloud2_to_xyz_array(cloud_msg):
    cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)
    cloud_arr = np.reshape(cloud_arr, (cloud_msg.width, int(cloud_msg.point_step / 4)))
    return cloud_arr[:, :3]


class TestDistortionCorrectionNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Init ROS at once
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS at once
        rclpy.shutdown()

    def setUp(self):
        # called when each test started
        self.node = rclpy.create_node("test_distortion_correction_node")
        # send static transform from map to base_link
        tf_msg = generate_static_transform_msg()
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.tf_broadcaster.sendTransform(tf_msg)
        self.imu_unused_callback_msg_buffer = []
        self.imu_used_callback_msg_buffer = []
        self.twist_publisher, self.imu_publisher, self.pointcloud_publisher = self.create_pub_sub()
        self.pointcloud_timestamp = rclpy.clock.Clock().now().to_msg()

    def tearDown(self):
        # called when each test finished
        self.node.destroy_node()

    def imu_unused_callback(self, msg: PointCloud2):
        self.imu_unused_callback_msg_buffer.append(msg)

    def imu_used_callback(self, msg: PointCloud2):
        self.imu_used_callback_msg_buffer.append(msg)

    def create_pub_sub(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Publishers
        twist_publisher = self.node.create_publisher(
            TwistWithCovarianceStamped,
            "/test/twist_with_covariance",
            10,
        )

        imu_publisher = self.node.create_publisher(
            Imu,
            "/test/imu/imu_data",
            10,
        )

        pointcloud_publisher = self.node.create_publisher(
            PointCloud2,
            "/test/lidar/top/pointcloud_raw",
            qos_profile=sensor_qos,
        )

        # clean subscriber
        self.imu_unused_callback_msg_buffer = []
        self.imu_used_callback_msg_buffer = []

        self.node.create_subscription(
            PointCloud2,
            "/test/lidar/top/imu_unused/pointcloud_rectified",
            self.imu_unused_callback,
            qos_profile=sensor_qos,
        )

        self.node.create_subscription(
            PointCloud2,
            "/test/lidar/top/imu_used/pointcloud_rectified",
            self.imu_used_callback,
            qos_profile=sensor_qos,
        )

        return twist_publisher, imu_publisher, pointcloud_publisher

    def test_normal_input_without_imu(self):
        """
        Test normal situation.

        input: distorted pointclouds, twist, imu.
        output: undistorted pointcloud
        """
        # wait for the node to be ready
        time.sleep(3)

        twist_msgs = generate_twist_msgs(self.pointcloud_timestamp)
        imu_msgs = generate_imu_msgs(self.pointcloud_timestamp)

        for i in range(len(twist_msgs)):
            self.twist_publisher.publish(twist_msgs[i])
            self.imu_publisher.publish(imu_msgs[i])

        pointcloud_msg = get_pointcloud_msg(True, False, self.pointcloud_timestamp)
        self.pointcloud_publisher.publish(pointcloud_msg)

        time.sleep(1)
        rclpy.spin_once(self.node, timeout_sec=1.0)

        # test withuout using imu
        self.assertEqual(
            len(self.imu_unused_callback_msg_buffer),
            1,
            "test_normal_input failed (not using imu): recieve more or less than one pointcloud.",
        )
        expected_pointcloud = np.array(
            [
                [1.0000000e01, 0.0000000e00, 0.0000000e00],
                [1.1712366e-01, 1.0000034e01, 0.0000000e00],
                [2.5999972e-01, 1.3518192e-04, 1.0000000e01],
                [2.0399988e01, 2.1381771e-02, 0.0000000e00],
                [5.0931960e-01, 2.0000475e01, 0.0000000e00],
                [6.9999874e-01, 8.1971986e-04, 2.0000000e01],
                [3.0859905e01, 7.6000042e-02, 0.0000000e00],
                [9.4795841e-01, 3.0001638e01, 0.0000000e00],
                [1.2199957e00, 2.4438177e-03, 3.0000000e01],
                [1.1356758e01, 1.0046270e01, 1.0000000e01],
            ],
            dtype=np.float32,
        )

        received_pointcloud = pointcloud2_to_xyz_array(self.imu_unused_callback_msg_buffer[0])
        self.assertTrue(
            np.allclose(received_pointcloud, expected_pointcloud, atol=1e-07),
            f"test_normal_input failed (not using imu): wrong undistorted pointcloud.\nExpected:\n{expected_pointcloud}\nReceived:\n{received_pointcloud}\nDifference:\n{ np.abs(received_pointcloud - expected_pointcloud)}",
        )

    def test_normal_input_with_imu(self):
        """
        Test normal situation.

        input: distorted pointclouds, twist, imu.
        output: undistorted pointcloud
        """
        # wait for the node to be ready
        time.sleep(3)

        twist_msgs = generate_twist_msgs(self.pointcloud_timestamp)
        imu_msgs = generate_imu_msgs(self.pointcloud_timestamp)

        for i in range(len(twist_msgs)):
            self.twist_publisher.publish(twist_msgs[i])
            self.imu_publisher.publish(imu_msgs[i])

        pointcloud_msg = get_pointcloud_msg(True, False, self.pointcloud_timestamp)
        self.pointcloud_publisher.publish(pointcloud_msg)

        time.sleep(1)
        rclpy.spin_once(self.node, timeout_sec=1.0)

        # test with using imu
        print("len(self.imu_used_callback_msg_buffer): ", len(self.imu_used_callback_msg_buffer))
        self.assertEqual(
            len(self.imu_used_callback_msg_buffer),
            1,
            "test_normal_input failed (using imu): recieve more or less than one pointcloud.",
        )
        expected_pointcloud = np.array(
            [
                [1.00000000e01, 0.00000000e00, 0.00000000e00],
                [1.14247441e-01, 1.00000620e01, 0.00000000e00],
                [2.59999633e-01, 2.24344447e-04, 1.00000000e01],
                [2.03999691e01, 3.49872075e-02, 0.00000000e00],
                [4.92061913e-01, 2.00007439e01, 0.00000000e00],
                [6.99997902e-01, 1.28470652e-03, 2.00000000e01],
                [3.08597984e01, 1.11163504e-01, 0.00000000e00],
                [9.10565615e-01, 3.00023651e01, 0.00000000e00],
                [1.21999240e00, 3.54156620e-03, 3.00000000e01],
                [1.13432636e01, 1.00609732e01, 1.00000000e01],
            ],
            dtype=np.float32,
        )

        received_pointcloud = pointcloud2_to_xyz_array(self.imu_used_callback_msg_buffer[0])
        self.assertTrue(
            np.allclose(received_pointcloud, expected_pointcloud, atol=1e-07),
            f"test_normal_input failed (using imu): wrong undistorted pointcloud.\nExpected:\n{expected_pointcloud}\nReceived:\n{received_pointcloud}\nDifference:\n{ np.abs(received_pointcloud - expected_pointcloud)}",
        )

    # Test:
    # send empty message (pointcloud)
    # send empty message (twist)
    # send empty message (imu)
    # send late message (twist)
    # send late message (imu)
