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
import std_srvs.srv


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['trigger.type'] = 'service'
    parameters['trigger.interface_type'] = 'std_srvs/srv/Trigger'
    parameters['trigger.service_name'] = '/trigger'
    parameters['trigger.buttons'] = [4]

    return generate_joy_test_description(parameters)


class TestJoyTeleopServiceTrigger(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        self.joy_msg.buttons[4] = int(not self.joy_msg.buttons[4])

    def test_trigger_service(self):
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

        srv = self.node.create_service(std_srvs.srv.Trigger, '/trigger', trigger)

        try:
            # Above we set the button to be used as '4', so here we set the '4' button active.
            self.joy_msg.buttons = [0, 0, 0, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for trigger service to complete')
            self.assertTrue(saw_trigger)
        finally:
            # Cleanup
            self.node.destroy_service(srv)
