# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Open Source Robotics Foundation
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop
import pytest
import rclpy
import std_msgs.msg


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['simple_message.type'] = 'topic'
    parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
    parameters['simple_message.topic_name'] = '/simple_message_type'
    parameters['simple_message.deadman_buttons'] = [2]
    parameters['simple_message.message_value.data.value'] = 'button2'

    return generate_joy_test_description(parameters)


class TestJoyTeleopTopicMessageValue(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        self.joy_msg.buttons[2] = int(not self.joy_msg.buttons[2])

    def test_simple_message(self):
        simple_message = None
        future = rclpy.task.Future()

        def receive_simple_message(msg):
            nonlocal simple_message
            nonlocal future
            simple_message = msg.data
            future.set_result(True)

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        simple_message_subscriber = self.node.create_subscription(
            std_msgs.msg.String,
            '/simple_message_type',
            receive_simple_message,
            qos,
        )

        try:
            # Above we set the button to be used as '2', so here we set the '2' button active.
            self.joy_msg.buttons = [0, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for simple_message topic to complete')
            self.assertEqual(simple_message, 'button2')
        finally:
            # Cleanup
            self.node.destroy_subscription(simple_message_subscriber)
