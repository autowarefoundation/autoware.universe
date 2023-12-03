#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# ROS 2 module imports
from std_msgs.msg import Int32, Float32 # Int32 and Float32 message classes
from geometry_msgs.msg import Point # Point message class
from sensor_msgs.msg import JointState, Imu, PointCloud2, Image # JointState, Imu, PointCloud2 and Image message classes
from nav_msgs.msg import Odometry # Odometry message class

# Python mudule imports
from attrdict import AttrDict # Mapping objects that allow their elements to be accessed both as keys and as attributes

################################################################################

# ROS 2 publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Vehicle data subscribers
        {'topic':'/autodrive/hunter_1/throttle_command', 'type': Float32, 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/hunter_1/steering_command', 'type': Float32, 'name': 'sub_steering_command'},
        # Traffic light data subscribers
        {'topic':'/autodrive/signal_1/command', 'type': Int32, 'name': 'sub_signal_1_command'},
        {'topic':'/autodrive/signal_2/command', 'type': Int32, 'name': 'sub_signal_2_command'},
        {'topic':'/autodrive/signal_3/command', 'type': Int32, 'name': 'sub_signal_3_command'},
        {'topic':'/autodrive/signal_4/command', 'type': Int32, 'name': 'sub_signal_4_command'}
    ],
    'publishers': [
        # Vehicle data publishers
        {'topic': '/autodrive/hunter_1/throttle', 'type': Float32, 'name': 'pub_throttle'},
        {'topic': '/autodrive/hunter_1/steering', 'type': Float32, 'name': 'pub_steering'},
        {'topic': '/autodrive/hunter_1/left_encoder', 'type': JointState, 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/hunter_1/right_encoder', 'type': JointState, 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/hunter_1/ips', 'type': Point, 'name': 'pub_ips'},
        {'topic': '/autodrive/hunter_1/imu', 'type': Imu, 'name': 'pub_imu'},
        {'topic': '/autodrive/hunter_1/odom', 'type': Odometry, 'name': 'pub_odom'},
        {'topic': '/autodrive/hunter_1/lidar', 'type': PointCloud2, 'name': 'pub_lidar'},
        {'topic': '/autodrive/hunter_1/front_camera', 'type': Image, 'name': 'pub_front_camera'},
        {'topic': '/autodrive/hunter_1/rear_camera', 'type': Image, 'name': 'pub_rear_camera'},
        # Traffic light data publishers
        {'topic': '/autodrive/signal_1/state', 'type': Int32, 'name': 'pub_signal_1_state'},
        {'topic': '/autodrive/signal_2/state', 'type': Int32, 'name': 'pub_signal_2_state'},
        {'topic': '/autodrive/signal_3/state', 'type': Int32, 'name': 'pub_signal_3_state'},
        {'topic': '/autodrive/signal_4/state', 'type': Int32, 'name': 'pub_signal_4_state'}
    ]
})

################################################################################

# Initialize vehicle control commands
throttle_command = 0.0 # [-1, 1]
steering_command = 0.0 # [-1, 1]

################################################################################

# Initialize traffic light control commands
signal_1_command = 1 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_2_command = 1 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_3_command = 2 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_4_command = 3 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]