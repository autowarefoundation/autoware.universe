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

import contextlib
import unittest

import launch
import launch_ros
import launch_testing
import launch_testing.markers
import pytest


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestJoyTeleopParameterFailures(unittest.TestCase):

    @classmethod
    def setUpClass(cls, launch_service, proc_info, proc_output):
        @contextlib.contextmanager
        def launch_joy_teleop(self, parameters):
            joy_teleop_node = launch_ros.actions.Node(
                package='joy_teleop',
                executable='joy_teleop',
                output='both',
                parameters=[parameters])

            with launch_testing.tools.launch_process(
                    launch_service, joy_teleop_node, proc_info, proc_output) as joy_teleop:
                yield joy_teleop

        cls.launch_joy_teleop = launch_joy_teleop

    def test_missing_type(self):
        parameters = {}
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.message_value.data.value'] = 'button2'

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("KeyError: 'type'" in joy_teleop_process.output)

    def test_invalid_type(self):
        parameters = {}
        parameters['simple_message.type'] = 'foo'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.message_value.data.value'] = 'button2'

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue('JoyTeleopException: unknown type' in joy_teleop_process.output)

    def test_no_buttons_or_axes(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.message_value.data.value'] = 'button2'

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue('JoyTeleopException: No buttons or axes configured for command'
                        in joy_teleop_process.output)

    def test_teleop_no_message_value_or_axis_mappings(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("JoyTeleopException: No 'message_value' or 'axis_mappings' configured "
                        'for command' in joy_teleop_process.output)

    def test_teleop_both_message_value_and_axis_mappings(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]
        parameters['simple_message.message_value.data.value'] = 'button2'
        parameters['simple_message.axis_mappings.linear-x.axis'] = 1
        parameters['simple_message.axis_mappings.linear-x.scale'] = 0.8
        parameters['simple_message.axis_mappings.linear-x.offset'] = 0.0

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("JoyTeleopException: Only one of 'message_value' or 'axis_mappings' "
                        'can be configured for command' in joy_teleop_process.output)

    def test_teleop_axis_mappings_missing_axis_or_button(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]
        parameters['simple_message.axis_mappings.linear-x.scale'] = 0.8
        parameters['simple_message.axis_mappings.linear-x.offset'] = 0.0

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue('must have an axis, button, or value' in joy_teleop_process.output)

    def test_teleop_axis_mappings_missing_offset(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]
        parameters['simple_message.axis_mappings.linear-x.axis'] = 1
        parameters['simple_message.axis_mappings.linear-x.scale'] = 0.8

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue('must have an offset' in joy_teleop_process.output)

    def test_teleop_axis_mappings_missing_scale(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]
        parameters['simple_message.axis_mappings.linear-x.axis'] = 1
        parameters['simple_message.axis_mappings.linear-x.offset'] = 0.0

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue('must have a scale' in joy_teleop_process.output)

    def test_teleop_invalid_message_fields(self):
        parameters = {}
        parameters['simple_message.type'] = 'topic'
        parameters['simple_message.interface_type'] = 'std_msgs/msg/String'
        parameters['simple_message.topic_name'] = '/simple_message_type'
        parameters['simple_message.deadman_buttons'] = [2]
        parameters['simple_message.message_value.foo.value'] = 'button2'

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("AttributeError: 'String' object has no attribute 'foo'"
                        in joy_teleop_process.output)

    def test_service_invalid_message_fields(self):
        parameters = {}
        parameters['trigger.type'] = 'service'
        parameters['trigger.interface_type'] = 'std_srvs/srv/Trigger'
        parameters['trigger.service_name'] = '/trigger'
        parameters['trigger.buttons'] = [4]
        parameters['trigger.service_request.foo'] = 5

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("AttributeError: 'Trigger_Request' object has no attribute 'foo'"
                        in joy_teleop_process.output)

    def test_action_invalid_message_fields(self):
        parameters = {}
        parameters['fibonacci.type'] = 'action'
        parameters['fibonacci.interface_type'] = 'action_tutorials_interfaces/action/Fibonacci'
        parameters['fibonacci.action_name'] = '/fibonacci'
        parameters['fibonacci.buttons'] = [2]
        parameters['fibonacci.action_goal'] = {'foo': 5}

        with self.launch_joy_teleop(parameters) as joy_teleop_process:
            self.assertTrue(joy_teleop_process.wait_for_shutdown(timeout=10))

        self.assertEqual(joy_teleop_process.exit_code, 1)
        self.assertTrue("AttributeError: 'Fibonacci_Goal' object has no attribute 'foo'"
                        in joy_teleop_process.output)
