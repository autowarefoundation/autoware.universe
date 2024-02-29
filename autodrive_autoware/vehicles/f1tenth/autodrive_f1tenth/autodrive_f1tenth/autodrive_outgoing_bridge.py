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
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from ament_index_python.packages import get_package_share_directory # Access package's shared directory path

# Python mudule imports
import numpy as np # Scientific computing
import configparser # Parsing shared configuration file(s)
import autodrive_f1tenth.config as config # AutoDRIVE Ecosystem ROS 2 configuration for F1TENTH vehicle

################################################################################

# Global declarations
global throttle_command, steering_command

# Initialize vehicle control commands
throttle_command = config.throttle_command
steering_command = config.steering_command

#########################################################
# ROS 2 SUBSCRIBER CALLBACKS
#########################################################

# VEHICLE DATA SUBSCRIBER CALLBACKS

def callback_throttle_command(throttle_command_msg):
    global throttle_command
    throttle_command = float(np.round(throttle_command_msg.data, 3))

def callback_steering_command(steering_command_msg):
    global steering_command
    steering_command = float(np.round(steering_command_msg.data, 3))

#########################################################
# AUTODRIVE ROS 2 OUTGOING BRIDGE INFRASTRUCTURE
#########################################################

def main():
    # Global declarations
    global throttle_command, steering_command
    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_outgoing_bridge = rclpy.create_node('autodrive_outgoing_bridge') # Create ROS 2 node
    qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    callbacks = {
        # Vehicle data subscriber callbacks
        '/autodrive/f1tenth_1/throttle_command': callback_throttle_command,
        '/autodrive/f1tenth_1/steering_command': callback_steering_command,
    } # Subscriber callback functions
    subscribers = [autodrive_outgoing_bridge.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile)
                   for e in config.pub_sub_dict.subscribers] # Subscribers
    subscribers # Avoid unused variable warning

    # Get package's shared directory path
    package_share_directory = get_package_share_directory('autodrive_f1tenth')

    # Recursive operations while node is alive
    while rclpy.ok():
        # Create and write data to shared config file
        api_config = configparser.ConfigParser()
        api_config['f1tenth_1'] = {'throttle_command': str(throttle_command),
                                   'steering_command': str(steering_command)
                                   }
        with open(package_share_directory+'/api_config.ini', 'w') as configfile:
            api_config.write(configfile)
        # Spin the node once
        rclpy.spin_once(autodrive_outgoing_bridge)
    
    autodrive_outgoing_bridge.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE Ecosystem ROS 2 outgoing bridge