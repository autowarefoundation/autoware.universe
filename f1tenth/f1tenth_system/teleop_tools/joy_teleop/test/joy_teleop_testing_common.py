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

import re
import unittest

import launch
import launch_ros
import launch_testing
import rclpy
import sensor_msgs.msg


def generate_joy_test_description(parameters):
    joy_teleop_node = launch_ros.actions.Node(
        package='joy_teleop',
        executable='joy_teleop',
        output='both',
        parameters=[parameters])

    return (
        launch.LaunchDescription([
            joy_teleop_node,
            launch_testing.actions.ReadyToTest(),
        ]), {}
    )


class TestJoyTeleop(unittest.TestCase):

    def publish_message(self):
        pass

    def setUp(self):
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        node_name = re.sub('(.)([A-Z]{1})', r'\1_\2', self.__class__.__name__).lower()
        self.node = rclpy.create_node(node_name, context=self.context)
        self.executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        self.joy_publisher = self.node.create_publisher(
            msg_type=sensor_msgs.msg.Joy,
            topic='joy',
            qos_profile=qos,
        )

        self.joy_msg = sensor_msgs.msg.Joy()
        self.joy_msg.header.stamp.sec = 0
        self.joy_msg.header.stamp.nanosec = 0
        self.joy_msg.header.frame_id = ''
        self.joy_msg.axes = []
        self.joy_msg.buttons = []

        self.publish_timer = self.node.create_timer(0.1, self.publish_message)

    def tearDown(self):
        self.node.destroy_timer(self.publish_timer)
        self.joy_publisher.destroy()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
