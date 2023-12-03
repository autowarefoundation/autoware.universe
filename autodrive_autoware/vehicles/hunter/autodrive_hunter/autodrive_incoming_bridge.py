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
import tf2_ros # ROS bindings for tf2 library to handle transforms
from std_msgs.msg import Int32, Float32, Header # Int32, Float32 and Header message classes
from geometry_msgs.msg import Point, TransformStamped # Point and TransformStamped message classes
from sensor_msgs.msg import JointState, Imu, PointCloud2, PointField, Image # JointState, Imu, PointCloud2, PointField and Image message classes
from nav_msgs.msg import Odometry # Odometry message class
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
from ament_index_python.packages import get_package_share_directory # Access package's shared directory path

# Python module imports
from cv_bridge import CvBridge, CvBridgeError # ROS bridge for opencv library to handle images
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # Handler for WebSocket messages and lifecycle events
import socketio # Socket.IO realtime client and server
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
from io import BytesIO # Manipulate bytes data in memory
from PIL import Image # Python Imaging Library's (PIL's) Image module
import configparser # Parsing shared configuration file(s)
import autodrive_hunter.config as config # AutoDRIVE Ecosystem ROS 2 configuration for Hunter SE vehicle

################################################################################

# Global declarations
global autodrive_incoming_bridge, cv_bridge, publishers
global throttle_command, steering_command
global signal_1_command, signal_2_command, signal_3_command, signal_4_command

# Initialize vehicle control commands
throttle_command = config.throttle_command
steering_command = config.steering_command

# Initialize traffic light control commands
signal_1_command = config.signal_1_command
signal_2_command = config.signal_2_command
signal_3_command = config.signal_3_command
signal_4_command = config.signal_4_command

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_int_msg(val):
    i = Int32()
    i.data = int(val)
    return i

def create_float_msg(val):
    f = Float32()
    f.data = float(val)
    return f

def create_joint_state_msg(joint_angle, joint_name, frame_id):
    js = JointState()
    js.header = Header()
    js.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    js.header.frame_id = frame_id
    js.name = [joint_name]
    js.position = [joint_angle]
    js.velocity = []
    js.effort = []
    return js

def create_point_msg(position):
    p = Point()
    p.x = position[0]
    p.y = position[1]
    p.z = position[2]
    return p

def create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration):
    imu = Imu()
    imu.header = Header()
    imu.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    imu.header.frame_id = 'imu'
    imu.orientation.x = orientation_quaternion[0]
    imu.orientation.y = orientation_quaternion[1]
    imu.orientation.z = orientation_quaternion[2]
    imu.orientation.w = orientation_quaternion[3]
    imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.angular_velocity.x = angular_velocity[0]
    imu.angular_velocity.y = angular_velocity[1]
    imu.angular_velocity.z = angular_velocity[2]
    imu.angular_velocity_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    imu.linear_acceleration.x = linear_acceleration[0]
    imu.linear_acceleration.y = linear_acceleration[1]
    imu.linear_acceleration.z = linear_acceleration[2]
    imu.linear_acceleration_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
    return imu

def create_odom_msg(position, orientation_quaternion, angular_velocity, linear_acceleration, last_timestamp=[None]):
    current_timestamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    if last_timestamp[0] is not None:
        dt = current_timestamp.sec - last_timestamp[0].sec
    else:
        dt = 0.0
    odom = Odometry()
    odom.header = Header()
    odom.header.stamp = current_timestamp
    odom.header.frame_id = 'world'
    # Pose
    odom.pose.pose.position.x = position[0]
    odom.pose.pose.position.y = position[1]
    odom.pose.pose.position.z = position[2]
    odom.pose.pose.orientation.x = orientation_quaternion[0]
    odom.pose.pose.orientation.y = orientation_quaternion[1]
    odom.pose.pose.orientation.z = orientation_quaternion[2]
    odom.pose.pose.orientation.w = orientation_quaternion[3]
    odom.pose.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0001] # Noise of sqrt(0.0001) = 0.01 m or rad
    # Twist
    if last_timestamp[0] is not None:
        linear_velocity = np.cumsum(linear_acceleration * dt, axis=0)
        odom.twist.twist.linear.x = linear_velocity[0]
        odom.twist.twist.linear.y = linear_velocity[1]
        odom.twist.twist.linear.z = linear_velocity[2]
    else:
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = angular_velocity[0]
    odom.twist.twist.angular.y = angular_velocity[1]
    odom.twist.twist.angular.z = angular_velocity[2]
    odom.twist.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0001] # Noise of sqrt(0.0001) = 0.01 m/s or rad/s
    last_timestamp[0] = current_timestamp
    return odom

def create_pointcloud_msg(lidar_pointcloud):
    pc = PointCloud2()
    pc.header = Header()
    pc.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    pc.header.frame_id = 'lidar'
    pc.height = 1 # [uint32] 2D structure of the point cloud (if the cloud is unordered height is 1)
    pc.point_step = 24 # [uint32] Length of a point in bytes
    pc.width = int(len(lidar_pointcloud)/pc.point_step) # [uint32] Length of the point cloud data
    pc.row_step = pc.width*pc.point_step # [uint32] Length of a row in bytes
    pc.data = lidar_pointcloud # [uint8[]] Actual pointcloud data with size (row_step*height)
    if pc.point_step == 24:
        pc.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                     PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
                     PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
                     ] # [PointField[]] Describes the channels and their layout in the binary data blob
    elif pc.point_step == 48:
        pc.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                     PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
                     PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
                     PointField(name='azimuth', offset=24, datatype=PointField.FLOAT32, count=1),
                     PointField(name='distance', offset=28, datatype=PointField.FLOAT32, count=1),
                     PointField(name='return_type', offset=32, datatype=PointField.UINT8, count=1),
                     PointField(name='time_stamp', offset=40, datatype=PointField.FLOAT64, count=1)
                     ] # [PointField[]] Describes the channels and their layout in the binary data blob
    pc.is_bigendian = False # [bool] True if the data is big-endian (BE)
    pc.is_dense = True # [bool] True if there are no invalid points
    return pc

def create_image_msg(image_array, frame_id):
    img = cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
    img.header = Header()
    img.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    img.header.frame_id = frame_id
    return img

def broadcast_transform(child_frame_id, parent_frame_id, position_tf, orientation_tf):
    tb = tf2_ros.TransformBroadcaster(autodrive_incoming_bridge)
    tf = TransformStamped()
    tf.header.stamp = autodrive_incoming_bridge.get_clock().now().to_msg()
    tf.header.frame_id = parent_frame_id
    tf.child_frame_id = child_frame_id
    tf.transform.translation.x = position_tf[0] # Pos X
    tf.transform.translation.y = position_tf[1] # Pos Y
    tf.transform.translation.z = position_tf[2] # Pos Z
    tf.transform.rotation.x = orientation_tf[0] # Quat X
    tf.transform.rotation.y = orientation_tf[1] # Quat Y
    tf.transform.rotation.z = orientation_tf[2] # Quat Z
    tf.transform.rotation.w = orientation_tf[3] # Quat W
    tb.sendTransform(tf)

#########################################################
# ROS 2 PUBLISHER FUNCTIONS
#########################################################

# VEHICLE DATA PUBLISHER FUNCTIONS

def publish_actuator_feedbacks(throttle, steering):
    publishers['pub_throttle'].publish(create_float_msg(throttle))
    publishers['pub_steering'].publish(create_float_msg(steering))

def publish_encoder_data(encoder_angles):
    publishers['pub_left_encoder'].publish(create_joint_state_msg(encoder_angles[0], "left_encoder", "left_encoder"))
    publishers['pub_right_encoder'].publish(create_joint_state_msg(encoder_angles[1], "right_encoder", "right_encoder"))

def publish_ips_data(position):
    publishers['pub_ips'].publish(create_point_msg(position))

def publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_imu'].publish(create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration))

def publish_odom_data(position, orientation_quaternion, angular_velocity, linear_acceleration):
    publishers['pub_odom'].publish(create_odom_msg(position, orientation_quaternion, angular_velocity, linear_acceleration))

def publish_lidar_pointcloud(lidar_pointcloud):
    publishers['pub_lidar'].publish(create_pointcloud_msg(lidar_pointcloud))

def publish_camera_images(front_camera_image, rear_camera_image):
    publishers['pub_front_camera'].publish(create_image_msg(front_camera_image, "front_camera"))
    publishers['pub_rear_camera'].publish(create_image_msg(rear_camera_image, "rear_camera"))

# TRAFFIC LIGHT DATA PUBLISHER FUNCTIONS

def publish_signal_states(signal_1_state, signal_2_state, signal_3_state, signal_4_state):
    publishers['pub_signal_1_state'].publish(create_int_msg(signal_1_state))
    publishers['pub_signal_2_state'].publish(create_int_msg(signal_2_state))
    publishers['pub_signal_3_state'].publish(create_int_msg(signal_3_state))
    publishers['pub_signal_4_state'].publish(create_int_msg(signal_4_state))

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    # Global declarations
    global autodrive_incoming_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    global signal_1_command, signal_2_command, signal_3_command, signal_4_command

    # Get package's shared directory path
    package_share_directory = get_package_share_directory('autodrive_hunter')

    # Wait for data to become available
    if data:
        # Try to read data from shared config file
        api_config = configparser.ConfigParser()
        try:
            api_config.read(package_share_directory+'/api_config.ini')
            # Update vehicle control commands
            throttle_command = float(api_config['hunter_1']['throttle_command'])
            steering_command = float(api_config['hunter_1']['steering_command'])
            # Update traffic light control commands
            signal_1_command = int(api_config['signal_1']['signal_command'])
            signal_2_command = int(api_config['signal_2']['signal_command'])
            signal_3_command = int(api_config['signal_3']['signal_command'])
            signal_4_command = int(api_config['signal_4']['signal_command'])
        # Pass if file cannot be read
        except:
            pass

        ########################################################################
        # VEHICLE DATA
        ########################################################################
        # Actuator feedbacks
        throttle = float(data["V1 Throttle"])
        steering = float(data["V1 Steering"])
        publish_actuator_feedbacks(throttle, steering)
        # Wheel encoders
        encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        publish_encoder_data(encoder_angles)
        # IPS
        position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        publish_ips_data(position)
        # IMU
        orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)
        # Odometry
        publish_odom_data(position, orientation_quaternion, angular_velocity, linear_acceleration)
        # Cooordinate transforms
        broadcast_transform("hunter_1", "world", position, orientation_quaternion) # Vehicle frame defined at center of rear axle
        broadcast_transform("left_encoder", "hunter_1", np.asarray([0.0, 0.26, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0))
        broadcast_transform("right_encoder", "hunter_1", np.asarray([0.0, -0.26, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0))
        broadcast_transform("ips", "hunter_1", np.asarray([0.04, 0.0, 0.358]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("imu", "hunter_1", np.asarray([0.04, 0.0, 0.358]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("lidar", "hunter_1", np.asarray([0.275, 0.0, 0.625]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        broadcast_transform("front_camera", "hunter_1", np.asarray([0.475, 0.0, 0.453]), np.asarray([0.0, 0.0871557, 0.0, 0.9961947]))
        broadcast_transform("rear_camera", "hunter_1", np.asarray([0.075, 0.0, 0.453]), np.asarray([-0.0871557, 0.0, 0.9961947, 0.0]))
        broadcast_transform("front_left_wheel", "hunter_1", np.asarray([0.55, 0.26, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))))
        broadcast_transform("front_right_wheel", "hunter_1", np.asarray([0.55, -0.26, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))))
        broadcast_transform("rear_left_wheel", "hunter_1", np.asarray([0.0, 0.26, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0))
        broadcast_transform("rear_right_wheel", "hunter_1", np.asarray([0.0, -0.26, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0))
        # LIDAR
        lidar_pointcloud = np.frombuffer(base64.b64decode(data["V1 LIDAR Pointcloud"]), dtype=np.uint8).tolist()
        publish_lidar_pointcloud(lidar_pointcloud)
        # Cameras
        front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        rear_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Rear Camera Image"]))))
        publish_camera_images(front_camera_image, rear_camera_image)

        ########################################################################
        # TRAFFIC LIGHT DATA
        ########################################################################
        # Traffic light states
        signal_1_state = int(data["TL1 State"])
        signal_2_state = int(data["TL2 State"])
        signal_3_state = int(data["TL3 State"])
        signal_4_state = int(data["TL4 State"])
        publish_signal_states(signal_1_state, signal_2_state, signal_3_state, signal_4_state)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Vehicle and traffic light control commands
        sio.emit('Bridge', data={'V1 Throttle': str(throttle_command), 'V1 Steering': str(steering_command),
                                 'TL1 State': str(signal_1_command), 'TL2 State': str(signal_2_command), 'TL3 State': str(signal_3_command), 'TL4 State': str(signal_4_command)})

#########################################################
# AUTODRIVE ROS 2 INCOMING BRIDGE INFRASTRUCTURE
#########################################################

def main():
    # Global declarations
    global autodrive_incoming_bridge, cv_bridge, publishers
    global throttle_command, steering_command
    global signal_1_command, signal_2_command, signal_3_command, signal_4_command

    # ROS 2 infrastructure
    rclpy.init() # Initialize ROS 2 communication for this context
    autodrive_incoming_bridge = rclpy.create_node('autodrive_incoming_bridge') # Create ROS 2 node
    qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    cv_bridge = CvBridge() # ROS bridge object for opencv library to handle image data
    publishers = {e.name: autodrive_incoming_bridge.create_publisher(e.type, e.topic, qos_profile)
                  for e in config.pub_sub_dict.publishers} # Publishers

    # Recursive operations while node is alive
    while rclpy.ok():
        app = socketio.WSGIApp(sio) # Create socketio WSGI application
        pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as a gevent WSGI server            
        rclpy.spin_once(autodrive_incoming_bridge) # Spin the node once
    
    autodrive_incoming_bridge.destroy_node() # Explicitly destroy the node
    rclpy.shutdown() # Shutdown this context

################################################################################

if __name__ == '__main__':
    main() # Call main function of AutoDRIVE Ecosystem ROS 2 incoming bridge