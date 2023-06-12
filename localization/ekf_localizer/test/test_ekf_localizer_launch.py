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
import yaml

from ament_index_python import get_package_share_directory
import launch
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.logging import get_logger
import launch_testing
import pytest

import rclpy

from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

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
        self.evaluation_time = 0.5  # 500ms

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    def set_initial_pose(self):
        pub_init_pose = self.test_node.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", 10)
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = 0.0
        init_pose.pose.pose.position.y = 0.0
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.0
        init_pose.pose.pose.orientation.w = 1.0
        pub_init_pose.publish(init_pose)
    
    def activate_node(self):
        cli_trigger = self.test_node.create_client(SetBool, "/trigger_node")
        request = SetBool.Request()
        request.data = True
        cli_trigger.call_async(request)

    def test_node_link(self):
        """
        Test node linkage.
        """
        # Trigger ekf_localizer to activate the node
        self.activate_node()

        # Send initial pose
        self.set_initial_pose()

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
