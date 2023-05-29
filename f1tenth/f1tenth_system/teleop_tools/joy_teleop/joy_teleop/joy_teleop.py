# -*- coding: utf-8 -*-
#
# Copyright (c) 2019 PAL Robotics SL.
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

import importlib
import typing

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from rosidl_runtime_py import set_message_fields
import sensor_msgs.msg


class JoyTeleopException(Exception):
    pass


def get_interface_type(type_name: str, interface_type: str) -> typing.Any:
    split = type_name.split('/')
    if len(split) != 3:
        raise JoyTeleopException("Invalid type_name '{}'".format(type_name))
    package = split[0]
    interface = split[1]
    message = split[2]
    if interface != interface_type:
        raise JoyTeleopException("Cannot use interface of type '{}' for an '{}'"
                                 .format(interface, interface_type))

    mod = importlib.import_module(package + '.' + interface_type)
    return getattr(mod, message)


def set_member(msg: typing.Any, member: str, value: typing.Any) -> None:
    ml = member.split('-')
    if len(ml) < 1:
        return
    target = msg
    for i in ml[:-1]:
        target = getattr(target, i)
    setattr(target, ml[-1], value)


class JoyTeleopCommand:

    def __init__(self, name: str, config: typing.Dict[str, typing.Any],
                 button_name: str, axes_name: str) -> None:
        self.buttons: typing.List[str] = []
        if button_name in config:
            self.buttons = config[button_name]
        self.axes: typing.List[str] = []
        if axes_name in config:
            self.axes = config[axes_name]

        if len(self.buttons) == 0 and len(self.axes) == 0:
            self.buttons = ['default']
            # raise JoyTeleopException("No buttons or axes configured for command '{}'".format(name))

        # Used to short-circuit the run command if there aren't enough buttons in the message.
        self.min_button = 0
        if len(self.buttons) > 0 and 'default' not in self.buttons:
            self.min_button = int(min(self.buttons))
        self.min_axis = 0
        if len(self.axes) > 0:
            self.min_axis = int(min(self.axes))

        # This can be used to "debounce" the message; if there are multiple presses of the buttons
        # or axes, the command may only activate on the first one until it toggles again.  But this
        # is a command-specific behavior, the base class only provides the mechanism.
        self.active = False

    def update_active_from_buttons_and_axes(self, joy_state: sensor_msgs.msg.Joy) -> None:
        self.active = False

        if (self.min_button is not None and len(joy_state.buttons) <= self.min_button) and \
           (self.min_axis is not None and len(joy_state.axes) <= self.min_axis):
            # Not enough buttons or axes, so it can't possibly be a message for this command.
            return

        for button in self.buttons:
            if button == 'default' and sum(joy_state.buttons) == 0:
                self.active = True
            elif button == 'default' and sum(joy_state.buttons) > 0:
                pass
            else:
                try:
                    self.active |= joy_state.buttons[button] == 1
                except IndexError:
                    # An index error can occur if this command is configured for multiple buttons
                    # like (0, 10), but the length of the joystick buttons is only 1.  Ignore these.
                    pass

        for axis in self.axes:
            try:
                self.active |= joy_state.axes[axis] == 1.0
            except IndexError:
                # An index error can occur if this command is configured for multiple buttons
                # like (0, 10), but the length of the joystick buttons is only 1.  Ignore these.
                pass


class JoyTeleopTopicCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node: Node) -> None:
        super().__init__(name, config, 'deadman_buttons', 'deadman_axes')

        self.name = name

        self.topic_type = get_interface_type(config['interface_type'], 'msg')

        # A 'message_value' is a fixed message that is sent in response to an activation.  It is
        # mutually exclusive with an 'axis_mapping'.
        self.msg_value = None
        if 'message_value' in config:
            msg_config = config['message_value']

            # Construct the fixed message and try to fill it in.  This message will be reused
            # during runtime, and has the side benefit of giving the user early feedback if the
            # config can't work.
            self.msg_value = self.topic_type()
            for target, param in msg_config.items():
                set_member(self.msg_value, target, param['value'])

        # An 'axis_mapping' takes data from one part of the message and scales and offsets it to
        # publish if an activation happens.  This is typically used to take joystick analog data
        # and republish it as a cmd_vel.  It is mutually exclusive with a 'message_value'.
        self.axis_mappings = {}
        if 'axis_mappings' in config:
            self.axis_mappings = config['axis_mappings']
            # Now check that the mappings have all of the required configuration.
            for mapping, values in self.axis_mappings.items():
                if 'axis' not in values and 'button' not in values and 'value' not in values:
                    raise JoyTeleopException("Axis mapping for '{}' must have an axis, button, "
                                             'or value'.format(name))

                if 'axis' in values:
                    if 'offset' not in values:
                        raise JoyTeleopException("Axis mapping for '{}' must have an offset"
                                                 .format(name))

                    if 'scale' not in values:
                        raise JoyTeleopException("Axis mapping for '{}' must have a scale"
                                                 .format(name))

        if self.msg_value is None and not self.axis_mappings:
            raise JoyTeleopException("No 'message_value' or 'axis_mappings' "
                                     "configured for command '{}'".format(name))
        if self.msg_value is not None and self.axis_mappings:
            raise JoyTeleopException("Only one of 'message_value' or 'axis_mappings' "
                                     "can be configured for command '{}'".format(name))

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        self.pub = node.create_publisher(self.topic_type, config['topic_name'], qos)

    def run(self, node: Node, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active based on buttons and axes.
        # 3.  If this command is currently not active, return without publishing.
        # 4.  If this is a msg_value, and the value of the previous active is the same as now,
        #     debounce and return without publishing.
        # 5.  In all other cases, publish.  This means that this is a msg_value and the button
        #     transitioned from 0 -> 1, or it means that this is an axis mapping and data should
        #     continue to be published without debouncing.

        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        if self.msg_value is not None and last_active == self.active:
            return

        if self.msg_value is not None:
            # This is the case for a static message.
            msg = self.msg_value
        else:
            # This is the case to forward along mappings.
            msg = self.topic_type()

            for mapping, values in self.axis_mappings.items():
                if 'axis' in values:
                    if len(joy_state.axes) > values['axis']:
                        val = joy_state.axes[values['axis']] * values.get('scale', 1.0) + \
                            values.get('offset', 0.0)
                    else:
                        node.get_logger().error('Joystick has only {} axes (indexed from 0),'
                                                'but #{} was referenced in config.'.format(
                                                    len(joy_state.axes), values['axis']))
                        val = 0.0
                elif 'button' in values:
                    if len(joy_state.buttons) > values['button']:
                        val = joy_state.buttons[values['button']] * values.get('scale', 1.0) + \
                            values.get('offset', 0.0)
                    else:
                        node.get_logger().error('Joystick has only {} buttons (indexed from 0),'
                                                'but #{} was referenced in config.'.format(
                                                    len(joy_state.buttons), values['button']))
                        val = 0.0
                elif 'value' in values:
                    # Pass on the value as its Python-implicit type
                    val = values.get('value')
                else:
                    node.get_logger().error(
                        'No Supported axis_mappings type found in: {}'.format(mapping))
                    val = 0.0

                set_member(msg, mapping, val)

        # If there is a stamp field, fill it with now().
        if hasattr(msg, 'header'):
            msg.header.stamp = node.get_clock().now().to_msg()

        self.pub.publish(msg)


class JoyTeleopServiceCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node: Node) -> None:
        super().__init__(name, config, 'buttons', 'axes')

        self.name = name

        service_name = config['service_name']

        service_type = get_interface_type(config['interface_type'], 'srv')

        self.request = service_type.Request()

        if 'service_request' in config:
            # Set the message fields in the request in the constructor.  This request will be used
            # during runtime, and has the side benefit of giving the user early feedback if the
            # config can't work.
            set_message_fields(self.request, config['service_request'])

        self.service_client = node.create_client(service_type, service_name)
        self.client_ready = False

    def run(self, node: Node, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active.
        # 3.  If this command is currently not active, return without calling the service.
        # 4.  Save off the current state of whether the service was ready.
        # 5.  Update whether the service is ready.
        # 6.  If the service is not currently ready, return without calling the service.
        # 7.  If the service was already ready, and the state of the button is the same as before,
        #     debounce and return without calling the service.
        # 8.  In all other cases, call the service.  This means that either this is a button
        #     transition 0 -> 1, or that the service became ready since the last call.

        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        last_ready = self.client_ready
        self.client_ready = self.service_client.service_is_ready()
        if not self.client_ready:
            return
        if last_ready == self.client_ready and last_active == self.active:
            return

        self.service_client.call_async(self.request)


class JoyTeleopActionCommand(JoyTeleopCommand):

    def __init__(self, name: str, config: typing.Dict[str, typing.Any], node: Node) -> None:
        super().__init__(name, config, 'buttons', 'axes')

        self.name = name

        action_type = get_interface_type(config['interface_type'], 'action')

        self.goal = action_type.Goal()

        if 'action_goal' in config:
            # Set the message fields for the goal in the constructor.  This goal will be used
            # during runtime, and has hte side benefit of giving the user early feedback if the
            # config can't work.
            set_message_fields(self.goal, config['action_goal'])

        action_name = config['action_name']

        self.action_client = ActionClient(node, action_type, action_name)
        self.client_ready = False

    def run(self, node: Node, joy_state: sensor_msgs.msg.Joy) -> None:
        # The logic for responding to this joystick press is:
        # 1.  Save off the current state of active.
        # 2.  Update the current state of active.
        # 3.  If this command is currently not active, return without calling the action.
        # 4.  Save off the current state of whether the action was ready.
        # 5.  Update whether the action is ready.
        # 6.  If the action is not currently ready, return without calling the action.
        # 7.  If the action was already ready, and the state of the button is the same as before,
        #     debounce and return without calling the action.
        # 8.  In all other cases, call the action.  This means that either this is a button
        #     transition 0 -> 1, or that the action became ready since the last call.

        last_active = self.active
        self.update_active_from_buttons_and_axes(joy_state)
        if not self.active:
            return
        last_ready = self.client_ready
        self.client_ready = self.action_client.server_is_ready()
        if not self.client_ready:
            return
        if last_ready == self.client_ready and last_active == self.active:
            return

        self.action_client.send_goal_async(self.goal)


class JoyTeleop(Node):
    """
    Generic joystick teleoperation node.

    Will not start without configuration, has to be stored in 'teleop' parameter.
    See config/joy_teleop.yaml for an example.
    """

    def __init__(self):
        super().__init__('joy_teleop', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.commands = []

        names = []

        for name, config in self.retrieve_config().items():
            if name in names:
                raise JoyTeleopException("command '{}' was duplicated".format(name))

            try:
                interface_group = config['type']

                if interface_group == 'topic':
                    self.commands.append(JoyTeleopTopicCommand(name, config, self))
                elif interface_group == 'service':
                    self.commands.append(JoyTeleopServiceCommand(name, config, self))
                elif interface_group == 'action':
                    self.commands.append(JoyTeleopActionCommand(name, config, self))
                else:
                    raise JoyTeleopException("unknown type '{interface_group}' "
                                             "for command '{name}'".format_map(locals()))
            except TypeError:
                # This can happen on parameters we don't control, like 'use_sim_time'.
                self.get_logger().debug('parameter {} is not a dict'.format(name))

            names.append(name)

        # Don't subscribe until everything has been initialized.
        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)
        self._subscription = self.create_subscription(
            sensor_msgs.msg.Joy, 'joy', self.joy_callback, qos)

    def retrieve_config(self):
        config = {}
        for param_name in sorted(self._parameters.keys()):
            pval = self.get_parameter(param_name).value
            self.insert_dict(config, param_name, pval)
        return config

    def insert_dict(self, dictionary: typing.Dict[str, typing.Any], key: str, value: str) -> None:
        split = key.partition(PARAMETER_SEPARATOR_STRING)
        if split[0] == key and split[1] == '' and split[2] == '':
            dictionary[key] = value
        else:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[2], value)

    def joy_callback(self, msg: sensor_msgs.msg.Joy) -> None:
        for command in self.commands:
            command.run(self, msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()

    try:
        rclpy.spin(node)
    except JoyTeleopException as e:
        node.get_logger().error(e.message)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
