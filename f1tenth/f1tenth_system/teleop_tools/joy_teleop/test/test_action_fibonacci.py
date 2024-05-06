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

import action_tutorials_interfaces.action
from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop
import pytest
import rclpy


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['fibonacci.type'] = 'action'
    parameters['fibonacci.interface_type'] = 'action_tutorials_interfaces/action/Fibonacci'
    parameters['fibonacci.action_name'] = '/fibonacci'
    parameters['fibonacci.buttons'] = [2]
    parameters['fibonacci.action_goal'] = {'order': 5}

    return generate_joy_test_description(parameters)


class TestJoyTeleopActionFibonacci(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)
        self.joy_msg.buttons[2] = int(not self.joy_msg.buttons[2])

    def test_simple_message(self):
        sequence = []
        future = rclpy.task.Future()

        def fibonacci_callback(goal_handle):
            nonlocal future

            sequence.append(0)
            sequence.append(1)
            for i in range(1, goal_handle.request.order):
                sequence.append(sequence[i] + sequence[i-1])

            goal_handle.succeed()
            result = action_tutorials_interfaces.action.Fibonacci.Result()
            result.sequence = sequence
            future.set_result(True)
            return result

        action_server = rclpy.action.ActionServer(
            self.node,
            action_tutorials_interfaces.action.Fibonacci,
            'fibonacci',
            fibonacci_callback)

        try:
            # Above we set the button to be used as '2', so here we set the '2' button active.
            self.joy_msg.buttons = [0, 0, 1]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for action to complete')
            self.assertEqual(sequence, [0, 1, 1, 2, 3, 5])
        finally:
            # Cleanup
            action_server.destroy()
