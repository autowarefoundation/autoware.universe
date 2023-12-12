#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    test_pose_estimator_arbiter_launch_file = os.path.join(
        get_package_share_directory("pose_estimator_arbiter"),
        "launch",
        "pose_estimator_arbiter.launch.xml",
    )

    pose_estimator_arbiter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_pose_estimator_arbiter_launch_file),
        launch_arguments={"pose_sources": "['ndt', 'yabloc', 'eagleye']"}.items(),
    )

    return launch.LaunchDescription(
        [
            pose_estimator_arbiter,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestPoseEstimatorManager(unittest.TestCase):
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

    def tearDown(self):
        self.test_node.destroy_node()

    def yabloc_callback(self, srv):
        pass

    def test_node_link(self):
        # The arbiter waits for the service to start, so here it instantiates a meaningless service server.
        self.test_node.create_service(SetBool, "/yabloc_suspend_srv", self.yabloc_callback)

        # Receive state
        msg_buffer = []
        self.test_node.create_subscription(
            String,
            "/pose_estimator_arbiter/debug/string",
            lambda msg: msg_buffer.append(msg.data),
            10,
        )

        # Wait until the node publishes some topic
        while len(msg_buffer) == 0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        # Check if the arbiter state is transitioning correctly
        self.assertEqual(msg_buffer[-1], "enable All\nlocalization is not initialized")

        # Change UNINITIALIZED -> INITIALIZED
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub_init_state = self.test_node.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            qos_profile,
        )
        init_state = LocalizationInitializationState()
        init_state._state = LocalizationInitializationState.INITIALIZED
        pub_init_state.publish(init_state)

        # Wait until the node publishes some topic
        end_time = time.time() + 1.0  # 1.5s
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if the arbiter state is transitioning correctly
        self.assertEqual(msg_buffer[-1], "enable All\nestimated pose has not been published yet")

        # Wait until the node publishes some topic
        end_time = time.time() + 1.0  # 1.5s
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if the arbiter state is transitioning correctly
        self.assertEqual(msg_buffer[-1], "enable All\nestimated pose has not been published yet")

        # Publish dummy estimated pose
        pub_pose_stamped = self.test_node.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_with_covariance",
            10,
        )
        pose_stamped = PoseWithCovarianceStamped()
        pub_pose_stamped.publish(pose_stamped)

        # Wait until the node publishes some topic
        end_time = time.time() + 1.5  # 1.5s
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if the arbiter state is transitioning correctly
        self.assertEqual(msg_buffer[-1], "enable YabLoc\npcd is not subscribed yet")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
