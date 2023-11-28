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
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['trigger.type'] = 'service'
    parameters['trigger.interface_type'] = 'std_srvs/srv/Trigger'
    parameters['trigger.service_name'] = '/trigger'
    parameters['trigger.buttons'] = [4]
    parameters['simple_message.type'] = 'topic'
    parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
    parameters['simple_message.topic_name'] = '/simple_message_type'
    parameters['simple_message.deadman_buttons'] = [2]
    parameters['simple_message.message_value.data.value'] = 'button2'

    return generate_joy_test_description(parameters)


class TestJoyTeleopServiceNotReady(TestJoyTeleop):

    def setUp(self):
        self.toggle = True
        super().setUp()

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        if self.toggle:
            self.joy_msg.buttons[2] = int(not self.joy_msg.buttons[2])

    def test_not_ready_service(self):
        # The point of this test is to ensure that joy_teleop can attach to
        # services that come up *after* it has come up.  The way it works is
        # that we start off only adding our subscriber to the topic.  We then
        # start publishing joystick data.  Once the simple topic fires once
        # (signalling that joy_teleop is up and ready to work), we then create
        # the service, and try a trigger.

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
            # We also set the '4' button for the service trigger; this shouldn't do anything
            # right now, but ensures that we test the not active -> active logic in joy_teleop.
            self.joy_msg.buttons = [0, 0, 1, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            assert future.done() and future.result(), \
                'Timed out waiting for simple_message topic to complete'
            self.assertTrue(simple_message == 'button2')

            # If we've made it to this point with no assertions, then we are ready
            # to setup the service and trigger.

            saw_trigger = False
            future = rclpy.task.Future()

            def trigger(request, response):
                nonlocal saw_trigger
                nonlocal future
                response.success = True
                response.message = 'service_response'
                saw_trigger = True
                future.set_result(True)

                return response

            self.toggle = False

            srv = self.node.create_service(std_srvs.srv.Trigger, '/trigger', trigger)

            try:
                self.executor.spin_until_future_complete(future, timeout_sec=10)

                # Check
                self.assertTrue(future.done() and future.result(),
                                'Timed out waiting for trigger service to complete')
                self.assertTrue(saw_trigger)
            finally:
                # Cleanup
                self.node.destroy_service(srv)
        finally:
            self.node.destroy_subscription(simple_message_subscriber)
