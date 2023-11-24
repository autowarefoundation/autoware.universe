#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
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
#  * Neither the name of PAL Robotics SL. nor the names of its
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
#
#
# Authors:
#   * Siegfried-A. Gevatter
#   * Jeremie Deray (artivis)

import curses

# For 'q' keystroke exit
import os
import signal
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header


class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Form a velocity.

        Take a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            # TODO(artivis) Why are those floats ??
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyTeleop(Node):

    def __init__(self, interface):
        super().__init__('key_teleop')

        self._interface = interface

        self._publish_stamped_twist = self.declare_parameter('twist_stamped_enabled', False).value

        if self._publish_stamped_twist:
            self._pub_cmd = self.create_publisher(TwistStamped, 'key_vel',
                                                  qos_profile_system_default)
        else:
            self._pub_cmd = self.create_publisher(Twist, 'key_vel', qos_profile_system_default)

        self._hz = self.declare_parameter('hz', 10).value

        self._forward_rate = self.declare_parameter('forward_rate', 0.8).value
        self._backward_rate = self.declare_parameter('backward_rate', 0.5).value
        self._rotation_rate = self.declare_parameter('rotation_rate', 1.0).value
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

    movement_bindings = {
        curses.KEY_UP:    (1,  0),
        curses.KEY_DOWN:  (-1,  0),
        curses.KEY_LEFT:  (0,  1),
        curses.KEY_RIGHT: (0, -1),
    }

    def run(self):
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            # TODO(artivis) use Rate once available
            time.sleep(1.0/self._hz)

    def _make_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _make_twist_stamped(self, linear, angular):
        twist_stamped = TwistStamped()
        header = Header()
        header.stamp = rclpy.clock.Clock().now().to_msg()
        header.frame_id = 'key_teleop'

        twist_stamped.header = header
        twist_stamped.twist.linear.x = linear
        twist_stamped.twist.angular.z = angular
        return twist_stamped

    def _set_velocity(self):
        now = self.get_clock().now()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < Duration(seconds=0.4):
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            # TODO(artivis) no rclpy.signal_shutdown ?
            os.kill(os.getpid(), signal.SIGINT)
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = self.get_clock().now()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Linear: %f, Angular: %f' % (self._linear, self._angular))
        self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()

        if self._publish_stamped_twist:
            twist = self._make_twist_stamped(self._linear, self._angular)
        else:
            twist = self._make_twist(self._linear, self._angular)

        self._pub_cmd.publish(twist)


def execute(stdscr):
    rclpy.init()

    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
