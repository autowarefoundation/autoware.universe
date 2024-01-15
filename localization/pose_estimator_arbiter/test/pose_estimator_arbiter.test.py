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
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool

# This test confirms that the point cloud is initially relayed by arbiter.
# Then, the state transitions to YabLoc mode and confirms that the point cloud is no longer relayed.


@pytest.mark.launch_test
def generate_test_description():
    test_pose_estimator_arbiter_launch_file = os.path.join(
        get_package_share_directory("pose_estimator_arbiter"),
        "launch",
        "pose_estimator_arbiter.launch.xml",
    )

    pose_estimator_arbiter = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_pose_estimator_arbiter_launch_file),
        launch_arguments={
            "pose_sources": "[ndt, yabloc, eagleye]",  # ignore artag now
            # Because artag helper requests parameters of the artag node, it is difficult to execute correctly.
            "input_pointcloud": "/sensing/lidar/top/pointcloud",
        }.items(),
    )

    return launch.LaunchDescription(
        [
            pose_estimator_arbiter,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestPoseEstimatorArbiter(unittest.TestCase):
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

    def spin_for(self, duration_sec):
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

    def yabloc_suspend_service_callback(self, srv):
        pass

    def publish_input_topics(self):
        self.pub_ndt_input.publish(PointCloud2())
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
        self.pub_yabloc_input.publish(Image())
        rclpy.spin_once(self.test_node, timeout_sec=0.1)

    def create_publishers_and_subscribers(self):
        # Publisher
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub_ndt_input = self.test_node.create_publisher(
            PointCloud2, "/sensing/lidar/top/pointcloud", qos_profile
        )
        self.pub_yabloc_input = self.test_node.create_publisher(
            Image, "/sensing/camera/traffic_light/image_raw", qos_profile
        )

        # Subscriber
        self.test_node.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud/relay",
            lambda msg: self.ndt_relayed.append(msg.header),
            qos_profile,
        )
        self.test_node.create_subscription(
            Image,
            "/sensing/camera/traffic_light/image_raw/yabloc_relay",
            lambda msg: self.yabloc_relayed.append(msg.header),
            qos_profile,
        )

    def test_node_link(self):
        # The arbiter waits for the service to start, so here it instantiates a meaningless service server.
        self.test_node.create_service(
            SetBool,
            "/localization/pose_estimator/yabloc/pf/yabloc_suspend_srv",
            self.yabloc_suspend_service_callback,
        )

        # Define subscription buffer
        self.ndt_relayed = []
        self.yabloc_relayed = []

        # Create publishers and subscribers
        self.create_publishers_and_subscribers()

        # Wait 0.5 second for node to be ready
        self.spin_for(0.5)

        # Publish dummy input topics
        for _ in range(10):
            self.publish_input_topics()
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Wait 1.0 second for all topics to be subscribed
        self.spin_for(1.0)

        # Confirm both topics are relayed
        print("ndt", len(self.ndt_relayed))
        print("yabloc", len(self.yabloc_relayed))
        self.assertGreater(len(self.ndt_relayed), 5)
        self.assertGreater(len(self.yabloc_relayed), 5)

        # Change UNINITIALIZED -> INITIALIZED
        pub_init_state = self.test_node.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        init_state = LocalizationInitializationState()
        init_state._state = LocalizationInitializationState.INITIALIZED
        pub_init_state.publish(init_state)

        # Publish dummy estimated pose
        pub_pose_stamped = self.test_node.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_with_covariance",
            10,
        )
        pub_pose_stamped.publish(PoseWithCovarianceStamped())

        # Because pointcloud_map is not broadcasted, the arbiter state must be following:
        #  - ndt: disabled
        #  - yabloc: enabled
        #  - eagleye: disabled
        #  - artag: disabled

        # Wait 1.0 second for the node state to change
        self.spin_for(1.0)

        # Clear buffer
        self.ndt_relayed = []
        self.yabloc_relayed = []

        # Publish dummy input topics
        for _ in range(10):
            self.publish_input_topics()
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Wait 0.5 second for all topics to be subscribed
        self.spin_for(0.5)

        # Confirm that yabloc topic is relayed and ndt topic is not relayed.
        print("ndt", len(self.ndt_relayed))
        print("yabloc", len(self.yabloc_relayed))
        self.assertGreater(len(self.yabloc_relayed), 5)
        self.assertEqual(len(self.ndt_relayed), 0)
        return


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
