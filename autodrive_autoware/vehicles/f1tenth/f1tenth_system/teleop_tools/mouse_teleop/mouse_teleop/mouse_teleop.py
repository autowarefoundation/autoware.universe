#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2015 Enrique Fernandez
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
#  * Neither the name of Enrique Fernandez nor the names of its
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
#   * Enrique Fernandez
#   * Jeremie Deray (artivis)

import signal
import tkinter

from geometry_msgs.msg import Twist, Vector3
import numpy
import rclpy
from rclpy.node import Node


class MouseTeleop(Node):

    def __init__(self):
        super().__init__('mouse_teleop')

        # Retrieve params:
        self._frequency = self.declare_parameter('frequency', 0.0).value
        self._scale = self.declare_parameter('scale', 1.0).value
        self._holonomic = self.declare_parameter('holonomic', False).value

        # Create twist publisher:
        self._pub_cmd = self.create_publisher(Twist, 'mouse_vel', 10)

        # Initialize twist components to zero:
        self._v_x = 0.0
        self._v_y = 0.0
        self._w = 0.0

        # Initialize mouse position (x, y) to None (unknown); it's initialized
        # when the mouse button is pressed on the _start callback that handles
        # that event:
        self._x = None
        self._y = None

        # Create window:
        self._root = tkinter.Tk()
        self._root.title('Mouse Teleop')

        # Make window non-resizable:
        self._root.resizable(0, 0)

        # Create canvas:
        self._canvas = tkinter.Canvas(self._root, bg='white')

        # Create canvas objects:
        self._canvas.create_arc(0, 0, 0, 0, fill='red', outline='red',
                                width=1, style=tkinter.PIESLICE, start=90.0, tag='w')
        self._canvas.create_line(0, 0, 0, 0, fill='blue', width=4, tag='v_x')

        if self._holonomic:
            self._canvas.create_line(0, 0, 0, 0, fill='blue', width=4, tag='v_y')

        # Create canvas text objects:
        self._text_v_x = tkinter.StringVar()
        if self._holonomic:
            self._text_v_y = tkinter.StringVar()
        self._text_w = tkinter.StringVar()

        self._label_v_x = tkinter.Label(self._root, anchor=tkinter.W, textvariable=self._text_v_x)
        if self._holonomic:
            self._label_v_y = tkinter.Label(
                self._root, anchor=tkinter.W, textvariable=self._text_v_y)
        self._label_w = tkinter.Label(self._root, anchor=tkinter.W, textvariable=self._text_w)

        if self._holonomic:
            self._text_v_x.set('v_x = %0.2f m/s' % self._v_x)
            self._text_v_y.set('v_y = %0.2f m/s' % self._v_y)
            self._text_w.set('w   = %0.2f deg/s' % self._w)
        else:
            self._text_v_x.set('v = %0.2f m/s' % self._v_x)
            self._text_w.set('w = %0.2f deg/s' % self._w)

        self._label_v_x.pack()
        if self._holonomic:
            self._label_v_y.pack()
        self._label_w.pack()

        # Bind event handlers:
        self._canvas.bind('<Button-1>', self._start)
        self._canvas.bind('<ButtonRelease-1>', self._release)

        self._canvas.bind('<Configure>', self._configure)

        if self._holonomic:
            self._canvas.bind('<B1-Motion>', self._mouse_motion_linear)
            self._canvas.bind('<Shift-B1-Motion>', self._mouse_motion_angular)

            self._root.bind('<Shift_L>', self._change_to_motion_angular)
            self._root.bind('<KeyRelease-Shift_L>', self._change_to_motion_linear)
        else:
            self._canvas.bind('<B1-Motion>', self._mouse_motion_angular)

        self._canvas.pack()

        # If frequency is positive, use synchronous publishing mode:
        if self._frequency > 0.0:
            # Create timer for the given frequency to publish the twist:
            period = 1.0 / self._frequency

            self._timer = self.create_timer(period, self._publish_twist)

        # Handle ctrl+c on the window
        self._root.bind('<Control-c>', self._quit)

        # Nasty polling-trick to handle ctrl+c in terminal
        self._root.after(50, self._check)
        signal.signal(2, self._handle_signal)

        # Start window event manager main loop:
        self._root.mainloop()

    def _quit(self, ev):
        self._root.quit()

    def __del__(self):
        self._root.quit()

    def _check(self):
        self._root.after(50, self._check)

    def _handle_signal(self, signum, frame):
        self._quit(None)

    def _start(self, event):
        self._x, self._y = event.y, event.x

        self._y_linear = self._y_angular = 0

        self._v_x = self._v_y = self._w = 0.0

    def _release(self, event):
        self._v_x = self._v_y = self._w = 0.0

        self._send_motion()

    def _configure(self, event):
        self._width, self._height = event.height, event.width

        self._c_x = self._height / 2.0
        self._c_y = self._width / 2.0

        self._r = min(self._height, self._width) * 0.25

    def _mouse_motion_linear(self, event):
        self._v_x, self._v_y = self._relative_motion(event.y, event.x)

        self._send_motion()

    def _mouse_motion_angular(self, event):
        self._v_x, self._w = self._relative_motion(event.y, event.x)

        self._send_motion()

    def _update_coords(self, tag, x0, y0, x1, y1):
        x0 += self._c_x
        y0 += self._c_y

        x1 += self._c_x
        y1 += self._c_y

        self._canvas.coords(tag, (x0, y0, x1, y1))

    def _draw_v_x(self, v):
        x = -v * float(self._width)

        self._update_coords('v_x', 0, 0, 0, x)

    def _draw_v_y(self, v):
        y = -v * float(self._height)

        self._update_coords('v_y', 0, 0, y, 0)

    def _draw_w(self, w):
        x0 = y0 = -self._r
        x1 = y1 = self._r

        self._update_coords('w', x0, y0, x1, y1)

        yaw = w * numpy.rad2deg(self._scale)

        self._canvas.itemconfig('w', extent=yaw)

    def _send_motion(self):

        self._draw_v_x(self._v_x)
        if self._holonomic:
            self._draw_v_y(self._v_y)
        self._draw_w(self._w)

        if self._holonomic:
            self._text_v_x.set('v_x = %0.2f m/s' % self._v_x)
            self._text_v_y.set('v_y = %0.2f m/s' % self._v_y)
            self._text_w.set('w   = %0.2f deg/s' % numpy.rad2deg(self._w))
        else:
            self._text_v_x.set('v = %0.2f m/s' % self._v_x)
            self._text_w.set('w = %0.2f deg/s' % numpy.rad2deg(self._w))

        v_x = self._v_x * self._scale
        v_y = self._v_y * self._scale
        w = self._w * self._scale

        lin = Vector3(x=v_x, y=v_y, z=0.0)
        ang = Vector3(x=0.0, y=0.0, z=w)

        twist = Twist(linear=lin, angular=ang)
        self._pub_cmd.publish(twist)

    def _publish_twist(self):
        self._send_motion()

    def _relative_motion(self, x, y):
        dx = self._x - x
        dy = self._y - y

        dx /= float(self._width)
        dy /= float(self._height)

        dx = max(-1.0, min(dx, 1.0))
        dy = max(-1.0, min(dy, 1.0))

        return dx, dy

    def _change_to_motion_linear(self, event):
        if self._y is not None:
            y = event.x

            self._y_angular = self._y - y
            self._y = self._y_linear + y

    def _change_to_motion_angular(self, event):
        if self._y is not None:
            y = event.x

            self._y_linear = self._y - y
            self._y = self._y_angular + y


def main():
    try:
        rclpy.init()

        node = MouseTeleop()

        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
