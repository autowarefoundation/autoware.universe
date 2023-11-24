#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# twist_mux: system_blackbox.py
#
# Copyright (c) 2020 PAL Robotics S.L. All rights reserved.
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


# Authors:
#   * Siegfried-A. Gevatter

import unittest

import rospy
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from rate_publishers import RatePublishers, TimeoutManager


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
        cls._publishers = RatePublishers()
        cls._vel1 = cls._publishers.add_topic('vel_1', Twist)
        cls._vel2 = cls._publishers.add_topic('vel_2', Twist)
        cls._vel3 = cls._publishers.add_topic('vel_3', Twist)

        cls._lock1 = cls._publishers.add_topic('lock_1', Bool)
        cls._lock2 = cls._publishers.add_topic('lock_2', Bool)

        cls._timeout_manager = TimeoutManager()
        cls._timeout_manager.add(cls._publishers)
        cls._timeout_manager.spin_thread()

    def tearDown(self):
        # Reset all topics.
        twist_msg = twist(0, 0)
        unlock = Bool(False)

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

    @classmethod
    def _vel_cmd(cls):
        # TODO(artivis) use rate once available
        time.sleep(cls.MESSAGE_TIMEOUT)
        # TODO wait_for_msg-like functionnality not yet available
        # https://github.com/ros2/rclcpp/issues/520
        return rospy.wait_for_message('cmd_vel_out', Twist,
                                      timeout=cls.MESSAGE_TIMEOUT)

    def test_empty(self):
        try:
            self._vel_cmd()
            self.fail('twist_mux should not be publishing without any input')
        except rospy.ROSException:
            pass

    def test_basic(self):
        t = twist(2.0)
        self._vel1.pub(t, rate=5)
        self.assertEqual(t, self._vel_cmd())

    def test_basic_with_priorities(self):
        t1 = twist(2.0)
        t2 = twist(0, 1.0)

        # Publish twist from input1 @ 3Hz, it should be used.
        self._vel1.pub(t1, rate=5)
        self.assertEqual(t1, self._vel_cmd())

        # Publish twist from input3, it should have priority
        # over the one from input1.
        self._vel3.pub(t2, rate=10)
        self.assertEqual(t2, self._vel_cmd())

        # Stop publishing input 3 and wait for it to timeout.
        # Speed should fall back to input 1.
        self._vel3.stop()
        rospy.sleep(0.5)  # input is 0.3 in .yaml file
        self.assertEqual(t1, self._vel_cmd())

# TODO: test limits, test timeouts, etc.


if __name__ == '__main__':
    import rostest
    PKG_NAME = 'twist_mux'
    TEST_NAME = '%s_system_blackbox_test' % PKG_NAME
    rospy.init_node(TEST_NAME)
    rostest.rosrun(PKG_NAME, TEST_NAME, TestTwistMux)
