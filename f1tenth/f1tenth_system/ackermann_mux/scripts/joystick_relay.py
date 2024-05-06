#! /usr/bin/python3
# -*- coding: utf-8 -*-

#
# twist_mux: joystick_relay.py
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
#
# Authors:
#   * Enrique Fernandez
#   * Siegfried-A. Gevatter
#   * Jeremie Deray


import rclpy
import numpy as np

from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyPriority, JoyTurbo
from visualization_msgs.msg import Marker


class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / \
                (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step=1):
        """
        Compute the velocity.

        Take a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class ServiceLikeActionServer(object):

    def __init__(self, node, action_name, action_type, callback):
        self._action_type = action_type
        self._callback = callback
        self._server = ActionServer(
            node,
            action_type,
            action_name,
            self._cb)

    def _cb(self, goal):
        self._callback()
        goal.succeed()
        return self._action_type().Result()


class VelocityControl:

    def __init__(self, node):
        self._node = node
        self._num_steps = self._node.declare_parameter('turbo/steps', 1)

        forward_min = self._node.declare_parameter(
            'turbo/linear_forward_min', 1.0)
        forward_max = self._node.declare_parameter(
            'turbo/linear_forward_max', 1.0)
        self._forward = Velocity(forward_min, forward_max, self._num_steps)

        backward_min = self._node.declare_parameter(
            'turbo/linear_backward_min', forward_min)
        backward_max = self._node.declare_parameter(
            'turbo/linear_backward_max', forward_max)
        self._backward = Velocity(backward_min, backward_max, self._num_steps)

        lateral_min = self._node.declare_parameter(
            'turbo/linear_lateral_min', 1.0)
        lateral_max = self._node.declare_parameter(
            'turbo/linear_lateral_max', 1.0)
        self._lateral = Velocity(lateral_min, lateral_max, self._num_steps)

        angular_min = self._node.declare_parameter('turbo/angular_min', 1.0)
        angular_max = self._node.declare_parameter('turbo/angular_max', 1.0)
        self._angular = Velocity(angular_min, angular_max, self._num_steps)

        default_init_step = np.floor((self._num_steps + 1)/2.0)
        init_step = self._node.declare_parameter(
            'turbo/init_step', default_init_step)

        if init_step < 0 or init_step > self._num_steps:
            self._init_step = default_init_step
            self._node.get_logger().warn('Initial step %d outside range [1, %d]!'
                                         ' Falling back to default %d' %
                                         (init_step, self._num_steps, default_init_step))
        else:
            self._init_step = init_step

        self.reset_turbo()

    def validate_twist(self, cmd):
        if cmd.linear.z or cmd.angular.x or cmd.angular.y:
            self._node.get_logger().error(
                'Joystick provided invalid values, only linear.x, '
                'linear.y and angular.z may be non-zero.')
            return False
        if abs(cmd.linear.x) > 1.0 or abs(cmd.linear.y) > 1.0 or abs(cmd.angular.z) > 1.0:
            self._node.get_logger().error(
                "Joystick provided invalid values (%d, %d, %d), not in [-1, 1] range."
                % (cmd.linear.x, cmd.linear.y, cmd.angular.z))
            return False
        return True

    def scale_twist(self, cmd):
        twist = Twist()
        if self.validate_twist(cmd):
            if cmd.linear.x >= 0:
                twist.linear.x = self._forward(
                    cmd.linear.x, self._current_step)
            else:
                twist.linear.x = self._backward(
                    cmd.linear.x, self._current_step)
            twist.linear.y = self._lateral(cmd.linear.y, self._current_step)
            twist.angular.z = self._angular(
                cmd.angular.z, self._current_angular_step)
        return twist

    def increase_turbo(self):
        if self._current_step < self._num_steps:
            self._current_step += 1
        self.increase_angular_turbo()

    def decrease_turbo(self):
        if self._current_step > 1:
            self._current_step -= 1
        self.decrease_angular_turbo()

    def increase_angular_turbo(self):
        if self._current_angular_step < self._num_steps:
            self._current_angular_step += 1

    def decrease_angular_turbo(self):
        if self._current_angular_step > 1:
            self._current_angular_step -= 1

    def reset_turbo(self):
        self._current_step = self._init_step
        self._current_angular_step = self._init_step


class TextMarker(object):

    def __init__(self, node, scale=1.0, z=0.0):
        # TODO latch
        self._pub = node.create_publisher(Marker, 'text_marker', 1)

        self._scale = scale
        self._z = z

        # Build marker
        self._marker = Marker()

        self._marker.id = 0
        self._marker.type = Marker.TEXT_VIEW_FACING

        self._marker.header.frame_id = "base_footprint"

        self._marker.pose.position.z = self._z

        self._marker.pose.orientation.w = 1.0

        self._marker.scale.z = self._scale

        self._marker.color.a = 1.0
        self._marker.color.r = 1.0
        self._marker.color.g = 1.0
        self._marker.color.b = 1.0

    def update(self, joystick_priority, add=True):
        if add:
            self._marker.action = Marker.ADD

            self._marker.text = "Manual" if joystick_priority else "Autonomous"
        else:
            self._marker.action = Marker.DELETE

        self._pub.publish(self._marker)


class JoystickRelay(Node):

    def __init__(self):
        super().__init__('joystick_relay')

        self._current_priority = self.declare_parameter('priority', True)
        self._velocity_control = VelocityControl()

        self._marker = TextMarker(self, 0.5, 2.0)

        self._pub_cmd = self.create_publisher(Twist, 'joy_vel_out', 1)
        self._subscriber = self.create_subscription(
            Twist, 'joy_vel_in', self._forward_cmd, 1)

        # TODO Latch
        self._pub_priority = self.create_publisher(Bool, 'joy_priority', 1)

        # Wait for subscribers and publish initial joy_priority:
        self._pub_priority.publish(self._current_priority)
        self._marker.update(self._current_priority)

        # Marker timer (required to update the marker when the robot doesn't receive velocities):
        self._timer = self.create_timer(1.0, self._timer_callback)

        # Action servers to change priority & the currently active turbo step.
        # We aren't using services because they aren't supported by joy_teleop.
        self._server_priority = ServiceLikeActionServer(
            self, 'joy_priority_action', JoyPriority,
            self._toggle_priority)
        self._server_increase = ServiceLikeActionServer(
            self, 'joy_turbo_increase', JoyTurbo,
            self._velocity_control.increase_turbo)
        self._server_decrease = ServiceLikeActionServer(
            self, 'joy_turbo_decrease', JoyTurbo,
            self._velocity_control.decrease_turbo)
        self._server_angular_increase = ServiceLikeActionServer(
            self, 'joy_turbo_angular_increase', JoyTurbo,
            self._velocity_control.increase_angular_turbo)
        self._server_angular_decrease = ServiceLikeActionServer(
            self, 'joy_turbo_angular_decrease', JoyTurbo,
            self._velocity_control.decrease_angular_turbo)
        self._server_reset = ServiceLikeActionServer(
            self, 'joy_turbo_reset', JoyTurbo,
            self._velocity_control.reset_turbo)

    def _forward_cmd(self, cmd):
        if self._current_priority:
            self._pub_cmd.publish(self._velocity_control.scale_twist(cmd))

        self._marker.update(self._current_priority)

    def _toggle_priority(self):
        self._current_priority = not self._current_priority
        self.get_logger().info("Toggled joy_priority, current status is: %s",
                               self._current_priority)
        self._pub_priority.publish(self._current_priority)
        self._marker.update(self._current_priority)

        # Reset velocity to 0:
        if self._current_priority:
            self._pub_cmd.publish(Twist())

    def _timer_callback(self, event):
        self._marker.update(self._current_priority)


def main(args=None):
    rclpy.init(args=args)

    node = JoystickRelay()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
