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


class TestJoyTeleopDebounce(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)

    def test_debounce(self):
        # This test works by having a subscription to the output topic.  Every
        # time the subscription is received, we increment a counter.  There is
        # also a timer callback that executes every 0.1 seconds, publishing the
        # the 'joy' message with button 2 on.  We then let the system run for
        # 5 seconds.  Due to debouncing, we should only ever receive one
        # subscription callback in that time.

        num_received_messages = 0

        def receive_simple_message(msg):
            nonlocal num_received_messages
            num_received_messages += 1

        simple_message_subscriber = self.node.create_subscription(
            std_msgs.msg.String,
            '/simple_message_type',
            receive_simple_message,
            1,
        )

        try:
            self.joy_msg.buttons = [0, 0, 1]

            # This spin will *always* wait the full timeout.  That's because we want
            # to ensure that no matter how many times the button is pressed, the
            # debouncing only lets a single message through.
            self.executor.spin_until_future_complete(rclpy.task.Future(), timeout_sec=5)

            # Check
            self.assertEqual(num_received_messages, 1)
        finally:
            # Cleanup
            self.node.destroy_subscription(simple_message_subscriber)
