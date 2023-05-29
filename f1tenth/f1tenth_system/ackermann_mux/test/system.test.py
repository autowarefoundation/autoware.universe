# Copyright 2019 Canonical, Ltd
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

from rate_publishers import RatePublishers, TimeoutManager
import os
import unittest
import sys

import launch
from launch.actions.execute_process import ExecuteProcess

import launch_ros.actions

import launch_testing

import time
import threading
from rclpy.executors import MultiThreadedExecutor

import rclpy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

sys.path.append(os.path.abspath(os.path.dirname(os.path.realpath(__file__))))


def generate_test_description():
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dir_path = os.path.dirname(os.path.realpath(__file__))

    parameters_file = os.path.join(
        dir_path, 'system_config.yaml'
    )

    twist_mux = launch_ros.actions.Node(
        package='twist_mux', executable='twist_mux',
        parameters=[parameters_file], env=proc_env)

    publisher = ExecuteProcess(
        cmd=['ros2 topic pub /lock_1 std_msgs/Bool "data: False" -r 20'],
        shell=True, env=proc_env
    )

    # system_blackbox = launch_ros.actions.Node(
    # package='twist_mux', node_executable='system_blackbox.py', env=proc_env)

    return launch.LaunchDescription([
        twist_mux,
        publisher,
        # system_blackbox,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ])


def twist(x=0.0, r=0.0):
    """Return a Twist for the given linear and rotation speed."""
    t = Twist()
    t.linear.x = x
    t.angular.z = r
    return t


class TestTwistMux(unittest.TestCase):

    # Maximum time (in seconds) that it may take for a message
    # to be received by the target node.
    MESSAGE_TIMEOUT = 0.3

    # Value (in seconds) >= the highest topic/lock timeout.
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):

        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)

        cls.node = rclpy.create_node(
            'node', namespace='ns', context=cls.context)

        # Aim at emulating a 'wait_for_msg'
        cls._subscription = cls.node.create_subscription(
            Twist, 'cmd_vel_out', cls._cb, 1)
        cls._msg = None

        cls.executor = MultiThreadedExecutor(
            context=cls.context, num_threads=2)
        cls.executor.add_node(cls.node)

        cls._publishers = RatePublishers(cls.context)
        cls._vel1 = cls._publishers.add_topic('vel_1', Twist)
        cls._vel2 = cls._publishers.add_topic('vel_2', Twist)
        cls._vel3 = cls._publishers.add_topic('vel_3', Twist)

        cls._lock1 = cls._publishers.add_topic('lock_1', Bool)
        cls._lock2 = cls._publishers.add_topic('lock_2', Bool)

        cls.executor.add_node(cls._vel1)
        cls.executor.add_node(cls._vel2)
        cls.executor.add_node(cls._vel3)
        cls.executor.add_node(cls._lock1)
        cls.executor.add_node(cls._lock2)

        cls._timeout_manager = TimeoutManager()
        cls._timeout_manager.add(cls._publishers)
        cls._timeout_manager.spin_thread()

        cls.exec_thread = threading.Thread(target=cls.executor.spin)
        cls.exec_thread.start()

    def _cb(self, msg):
        self._msg = msg

    def _wait(self, timeout):
        start = self.node.get_clock().now()
        self._msg = None
        while (timeout > ((self.node.get_clock().now() - start).nanoseconds / 1e9)):
            if self._msg is not None:
                return self._msg
            time.sleep(0.01)
        return self._msg

    def tearDown(self):
        # Reset all topics.
        twist_msg = twist(0.0, 0.0)
        unlock = Bool()
        unlock.data = False

        self._vel1.pub(twist_msg)
        self._vel2.pub(twist_msg)
        self._vel3.pub(twist_msg)

        self._lock1.pub(unlock)
        self._lock2.pub(unlock)

        # Wait for previously published messages to time out,
        # since we aren't restarting twist_mux.
        #
        # This sleeping time must be higher than any of the
        # timeouts in system_test_config.yaml.
        #
        # TODO(artivis) use rate once available
        time.sleep(self.MESSAGE_TIMEOUT + self.TOPIC_TIMEOUT)

        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    @classmethod
    def _vel_cmd(cls):
        # TODO(artivis) use rate once available
        time.sleep(cls.MESSAGE_TIMEOUT)
        # TODO wait_for_msg-like functionnality not yet available
        # https://github.com/ros2/rclcpp/issues/520
        return cls._wait(cls, cls.MESSAGE_TIMEOUT)

    def test_empty(self):
        try:
            self._vel_cmd()
            self.fail('twist_mux should not be publishing without any input')
        except Exception:
            e = sys.exc_info()[0]
            print(e)
            pass

    def test_basic(self):
        t = twist(2.0)
        self._vel1.pub(t, rate=5)
        self.assertEqual(t, self._vel_cmd())

#    def test_basic_with_priorities(self):
#        t1 = twist(2.0)
#        t2 = twist(0.0, 1.0)

#        # Publish twist from input1 @ 3Hz, it should be used.
#        self._vel1.pub(t1, rate=5)
#        self.assertEqual(t1, self._vel_cmd())

#        # Publish twist from input3, it should have priority
#        # over the one from input1.
#        # self._vel3.pub(t2, rate=10)
#        self.assertEqual(t2, self._vel_cmd())

#        # Stop publishing input 3 and wait for it to timeout.
#        # Speed should fall back to input 1.
#        # self._vel3.stop()
#        time.sleep(0.5)  # input is 0.3 in .yaml file
#        self.assertEqual(t1, self._vel_cmd())


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(self.proc_info)
