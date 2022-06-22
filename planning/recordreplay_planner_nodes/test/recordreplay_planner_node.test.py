# Copyright 2020 Embotech AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file contains tests for the record and replay behavior of the containing node.
# I'll start by conceptually documenting the tests I want to add, then implement them.

import unittest

import ament_index_python
import launch
import launch_ros.actions
import launch_testing
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from autoware_auto_planning_msgs.action import RecordTrajectory, ReplayTrajectory
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import VehicleKinematicState

import subprocess
import time


# Class to publish some dummy states
class MockStatePublisher(Node):
    def __init__(self):
        super().__init__("MockStatePublisher")
        self.publisher_ = self.create_publisher(
            VehicleKinematicState, "vehicle_kinematic_state", 10
        )

    def publish_a_state(self):
        msg = VehicleKinematicState()
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing ego state...")


# Class to listen for trajectories being published and count them
class MockTrajectoryMonitor(Node):
    def __init__(self):
        super().__init__("MockTrajectoryMonitor")
        self.subscription_ = self.create_subscription(
            Trajectory, "trajectory", self.listener_callback, 10
        )
        self.trajectories_seen = 0

    def listener_callback(self, msg):
        self.get_logger().info('Received: "{}"'.format(msg))
        self.trajectories_seen += 1


class MockActionCaller(Node):
    def __init__(self, node_name, action_type, action_name):
        super().__init__(node_name)
        self._action_client = ActionClient(self, action_type, action_name)
        self.action_type = action_type

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._goal_handle = goal_handle

    def feedback_callback(self, feedback):
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.feedback.sequence)
        )

    def manual_cancel(self):
        self.get_logger().info("Canceling goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)
        return cancel_future

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending goal request...")
        goal_msg = self.action_type.Goal()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future


def generate_test_description():
    test_nodes = launch_ros.actions.Node(
        package="recordreplay_planner_nodes",
        executable="recordreplay_planner_node_exe",
        name="recordreplay_planner",
        parameters=[
            "{}/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "recordreplay_planner_nodes"
                )
            )
        ],
    )

    # integration test
    ld = launch.LaunchDescription(
        [
            test_nodes,
            launch_testing.actions.ReadyToTest(),
        ]
    )

    # An array of all the checkers to be enumerated by the tests
    return ld


def helper_pub_topic_one(name: str, msgtype: str):
    return subprocess.run(["ros2", "topic", "pub", name, msgtype, "-1"])


# Test: Check if "happy case" of recording, then replaying works
class TestBasicUsage(unittest.TestCase):
    def test_happy_case_works(self):
        # ---- Recording
        # - Start recordreplay_planner_node exe (done in launch description)

        # - Start a recording action by sending a goal
        mock_record_action_caller = MockActionCaller(
            "MockRecordCaller", RecordTrajectory, "recordtrajectory"
        )

        # - Send goal, then wait for the goal sending to complete
        send_goal_future = mock_record_action_caller.send_goal()
        rclpy.spin_until_future_complete(mock_record_action_caller, send_goal_future)
        rclpy.spin_once(mock_record_action_caller, timeout_sec=2)  # apparently needed

        # - Publish a few VehicleKinematicState messages
        mock_publisher = MockStatePublisher()
        for k in range(3):
            time.sleep(0.1)
            mock_publisher.publish_a_state()  # FIXME(s.me) This does not appear to get seen
            helper_pub_topic_one(  # This does get seen
                "/vehicle_kinematic_state",
                "autoware_auto_vehicle_msgs/msg/VehicleKinematicState",
            )

        # - Cancel recording action, then wait for the cancel action to complete
        cancel_future = mock_record_action_caller.manual_cancel()
        rclpy.spin_until_future_complete(mock_record_action_caller, cancel_future)
        rclpy.spin_once(mock_record_action_caller, timeout_sec=2)  # apparently needed

        # ---- Replaying
        # - Listen on the specified trajectory topic, storing to memory
        mock_listener = MockTrajectoryMonitor()

        # - Send it a "start replaying" action request
        mock_replay_action_caller = MockActionCaller(
            "MockReplayCaller", ReplayTrajectory, "replaytrajectory"
        )
        # - Send goal, wait for the goal sending to complete
        send_goal_future = mock_replay_action_caller.send_goal()
        rclpy.spin_until_future_complete(mock_replay_action_caller, send_goal_future)
        rclpy.spin_once(mock_replay_action_caller, timeout_sec=2)  # apparently needed

        # - Publish a few VehicleKinematicState messages
        for k in range(3):
            time.sleep(0.1)
            # FIXME(s.me) This does not appear to get seen by recordreplay node
            mock_publisher.publish_a_state()
            helper_pub_topic_one(  # This does get seen
                "/vehicle_kinematic_state",
                "autoware_auto_vehicle_msgs/msg/VehicleKinematicState",
            )
            time.sleep(0.1)

            # Spin the recording a couple of times, otherwise it'll not reliably process the
            # trajectory. FIXME(s.me) this has to be done more systematically, by for example
            # having the listener in the launch description itself so it just spins by itself.
            for kk in range(3):
                rclpy.spin_once(mock_listener, timeout_sec=0.2)

        # - Cancel replaying action, then wait for the cancellation to complete
        cancel_future = mock_replay_action_caller.manual_cancel()
        rclpy.spin_until_future_complete(mock_replay_action_caller, cancel_future)
        rclpy.spin_once(mock_replay_action_caller, timeout_sec=2)  # apparently needed

        # - Verify that the replayed trajectories behaved as expected
        # TODO(s.me): Make the mock_listener record what it sees and verify it matches
        # expectations.
        self.assertEqual(mock_listener.trajectories_seen, 3)


# TODO(s.me): Test: Check if an additional record action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Use ros2 commandline to send it a "start recording" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if an additional replay action is rejected if one is already running
# - Start recordreplay_planner_node exe
# - Record a bit of trajectory like in happy case test
# - Use ros2 commandline to send it a "start replaying" action request
# - Attempt to start a second start, verify this is rejected

# TODO(s.me): Test: Check if replay stops when the trajectory being put out becomes empty.
# This is not implemented in the actual code yet - maybe not stopping but just giving out
# the last one-state trajectory is a better idea.
