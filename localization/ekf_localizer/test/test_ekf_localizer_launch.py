#!/usr/bin/env python3

# Copyright 2022 TIER IV, Inc.
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

import os
import time
import unittest

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
from launch_ros.actions import Node
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from std_srvs.srv import SetBool
import yaml

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    test_ekf_localizer_launch_file = os.path.join(
        get_package_share_directory("ekf_localizer"),
        "launch",
        "ekf_localizer.launch.xml",
    )
    ekf_localizer = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_ekf_localizer_launch_file),
    )

    return launch.LaunchDescription(
        [
            ekf_localizer,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


# class TestManager(Node):
#     def __init__(self):
#         super().__init__('test_manager')
#         self.msg_buffer = []
#         self.sub_odom = self.create_subscription(
#             Odometry, "/ekf_odom", lambda msg: self.msg_buffer.append(msg), 10
#         )

#         self.cli = self.create_client(SetBool, '/trigger_node')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SetBool.Request()

#     def send_request(self):
#         self.req.data = True
#         self.future = self.cli.call_async(self.req)


# class TestEKFLocalizer(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         # Initialize the ROS context for the test node
#         rclpy.init()

#     @classmethod
#     def tearDownClass(cls):
#         # Shutdown the ROS context
#         rclpy.shutdown()

#     def setUp(self):
#         # Create a ROS node for tests
#         self.test_node = TestManager()
#         self.evaluation_time = 0.2  # 200ms

#     def tearDown(self):
#         self.test_node.destroy_node()

#     @staticmethod
#     def print_message(stat):
#         logger.debug("===========================")
#         logger.debug(stat)

#     def test_node_link(self):
#         """
#         Test node linkage.
#         """
#         # Trigger ekf_localizer to activate the node
#         self.test_node.send_request()
#         while rclpy.ok():
#             rclpy.spin_once(self.test_node)
#             if self.test_node.future.done():
#                 try:
#                     response = self.test_node.future.result()
#                 except Exception as e:
#                     self.test_node.get_logger().info(
#                         'Service call failed %r' % (e,))
#                 break

#         # Test init state.
#         # Wait until the talker transmits two messages over the ROS topic
#         end_time = time.time() + self.evaluation_time
#         while time.time() < end_time:
#             rclpy.spin_once(self.test_node, timeout_sec=0.1)

#         # Check if the EKF outputs some Odometry
#         self.assertTrue(len(self.msg_buffer) > 0)

class TestEKFLocalizer(unittest.TestCase):
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
        self.evaluation_time = 0.2  # 200ms

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    def test_node_link(self):
        """
        Test node linkage.
        """
        # Trigger ekf_localizer to activate the node
        cli_trigger = self.test_node.create_client(SetBool, "/trigger_node")
        while not cli_trigger.wait_for_service(timeout_sec=1.0):
            continue

        request = SetBool.Request()
        request.data = True
        future = cli_trigger.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future)

        if future.result() is not None:
            self.test_node.get_logger().info('Result of bool service: %s' % future.result().message)
        else:
            self.test_node.get_logger().error('Exception while calling service: %r' % future.exception())

        # Send initial pose
        pub_init_pose = self.test_node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", 10
        )
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        pub_init_pose.publish(init_pose)

        # Receive Odometry
        msg_buffer = []
        self.test_node.create_subscription(
            Odometry, "/ekf_odom", lambda msg: msg_buffer.append(msg), 10
        )

        # Test init state.
        # Wait until the talker transmits two messages over the ROS topic
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if the EKF outputs some Odometry
        self.assertTrue(len(msg_buffer) > 0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
