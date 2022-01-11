# Copyright 2022 Tier IV, Inc.
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
from time import sleep
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from std_msgs.msg import String


@pytest.mark.launch_test
def generate_test_description():
    test_topic_state_monitor_launch_file = os.path.join(
        get_package_share_directory("topic_state_monitor"),
        "test",
        "launch",
        "test_topic_state_monitor.launch.xml",
    )
    topic_state_monitor = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_topic_state_monitor_launch_file),
    )

    return launch.LaunchDescription(
        [
            topic_state_monitor,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestTopicStateMonitor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")
        self.evaluation_time = 3.0

    def tearDown(self):
        self.test_node.destroy_node()

    def test_topic_state_monitor(self):
        """
        Test that the topic_state_monitor node is able to publish a
        DiagnosticArray message.
        """

        pub_topic = self.test_node.create_publisher(String, "/test_topic", 10)
        msg = String(data="test_message")

        msg_buffer = []
        self.test_node.create_subscription(
            DiagnosticArray, "/diagnostics", lambda msg: msg_buffer.append(msg), 10
        )

        # Wait until the talker transmits two messages over the ROS topic
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            pub_topic.publish(msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if len(msg_buffer) > 0:
                break

        # Check that the message has the correct structure
        self.assertGreater(len(msg_buffer), 0)
        self.assertEqual(
            msg_buffer[0].status[0].name, "topic_state_monitor_test: test_topic_status"
        )
        self.assertEqual(msg_buffer[0].status[0].message, "OK")
        self.assertEqual(msg_buffer[0].status[0].level, DiagnosticStatus.OK)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """
        Test process exit code.

        Check that all processes in the launch (in this case, there's just one) exit
        with code 0
        """
        launch_testing.asserts.assertExitCodes(proc_info)
