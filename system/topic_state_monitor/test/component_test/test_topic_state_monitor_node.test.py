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
from time import sleep
import pytest
import unittest
import rclpy

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource



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

    def test_topic_state_monitor(self):
        # sleep(20.0)
        self.assertTrue(True)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """
        Test process exit code.

        Check that all processes in the launch (in this case, there's just one) exit
        with code 0
        """
        launch_testing.asserts.assertExitCodes(proc_info)
