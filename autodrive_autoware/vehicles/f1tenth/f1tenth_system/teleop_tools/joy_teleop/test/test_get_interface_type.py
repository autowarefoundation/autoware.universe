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

from joy_teleop.joy_teleop import get_interface_type
from joy_teleop.joy_teleop import JoyTeleopException

import pytest

import sensor_msgs.msg
import std_srvs.srv
import test_msgs.action


def test_message():
    interface_type = get_interface_type('sensor_msgs/msg/Joy', 'msg')
    msg = interface_type()
    assert isinstance(msg, sensor_msgs.msg.Joy)


def test_service():
    interface_type = get_interface_type('std_srvs/srv/Trigger', 'srv')
    srv = interface_type.Request()
    assert isinstance(srv, std_srvs.srv.Trigger.Request)


def test_action():
    interface_type = get_interface_type('test_msgs/action/Fibonacci', 'action')
    action = interface_type.Goal()
    assert isinstance(action, test_msgs.action.Fibonacci.Goal)


def test_bad_message():
    with pytest.raises(JoyTeleopException):
        get_interface_type('sensor_msgs/msg', 'msg')


def test_invalid_type():
    with pytest.raises(JoyTeleopException):
        get_interface_type('sensor_msgs/msg/Joy', 'ms')


def test_bad_import():
    with pytest.raises(ModuleNotFoundError):
        get_interface_type('foo_msgs/msg/Bar', 'msg')
