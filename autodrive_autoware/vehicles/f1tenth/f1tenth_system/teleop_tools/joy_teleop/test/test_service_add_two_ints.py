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

import example_interfaces.srv
from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['addtwoints.type'] = 'service'
    parameters['addtwoints.interface_type'] = 'example_interfaces/srv/AddTwoInts'
    parameters['addtwoints.service_name'] = '/addtwoints'
    parameters['addtwoints.buttons'] = [4]
    parameters['addtwoints.service_request.a'] = 5
    parameters['addtwoints.service_request.b'] = 4

    return generate_joy_test_description(parameters)


class TestJoyTeleopServiceAddTwoInts(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        self.joy_msg.buttons[4] = int(not self.joy_msg.buttons[4])

    def test_addtwoints_service(self):
        service_result = None
        future = rclpy.task.Future()

        def addtwoints(request, response):
            nonlocal service_result
            nonlocal future

            service_result = request.a + request.b
            response.sum = service_result
            future.set_result(True)

            return response

        srv = self.node.create_service(example_interfaces.srv.AddTwoInts, '/addtwoints',
                                       addtwoints)

        try:
            # Above we set the button to be used as '4', so here we set the '4' button active.
            self.joy_msg.buttons = [0, 0, 0, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for addtwoints service to complete')
            self.assertEqual(service_result, 9)
        finally:
            # Cleanup
            self.node.destroy_service(srv)
