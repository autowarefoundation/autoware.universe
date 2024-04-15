import pickle
from typing import Callable
# import smbus
from dora import DoraStatus
import serial
from typing import Callable
from dora import DoraStatus
from scipy.spatial.transform import Rotation as R
import math
import pickle
import pyarrow as pa
import numpy as np
import dora
import random
import json
from json import *
import time
serial_port = "/dev/ttyUSB1"
serial_baud = 115200
class Operator:
    """
    Template docstring
    """

    def __init__(self):
        """Called on initialisation"""
        self.data_length = 40
        self.imu_yaw_calibration = 0.0
        self.roll=0
        self.pitch=0
        self.yaw=0
        self.seq=0
        self.euler=[0,0,0]
        self.accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
        self.degrees2rad = math.pi/180.0
        self.header = {'seq':0.0,'frame_id': 'map', 'stamp': 0.0,}
        self.accel_data = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_data = {'x': 0, 'y': 0, 'z': 0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0,'w':0.0}
        self.IMU = None
         # Create a ROS2 Context
        self.ros2_context = dora.experimental.ros2_bridge.Ros2Context()
        self.ros2_node = self.ros2_context.new_node(
            "turtle_teleop",
            "/ros2_demo_imu",
            dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
        )

        # Define a ROS2 QOS
        self.topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
            reliable=True, max_blocking_time=0.1
        )

        # Create a publisher to imu topic
        self.turtle_imu_topic = self.ros2_node.create_topic(
            "/turtle1/imu", "sensor_msgs::Imu", self.topic_qos
        )
        self.imu_writer = self.ros2_node.create_publisher(self.turtle_imu_topic)
        """
        打开串口读数据，校验、解析后，send_out解析得到的消息类型
        # """
        # 打开串口读取数据
        try:
            self.IMU = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=3)
             # if self.IMU and self.IMU.is_open:
            #     self.IMU.close()
            # self.IMU.open()
            # self.tmp = self.IMU.write(('\xA5\x5A\x04\x02\x06\xAA' + chr(13)).encode())
            self.IMU.close()
            time.sleep(0.2)
            self.IMU.open()
            # self.IMU.flushInput() 
            # time.sleep(0.5)
            self.tmp = self.IMU.write(b'\xA5\x5A\x04\x01\x05\xAA\n')

        except serial.SerialException as ex:
            print(
                "Could not open serial port: I/O error({0}): {1}".format(
                    ex.errno, ex.strerror
                )
            )

 
 
    def euler2quaternion(self,euler):
        r = R.from_euler('xyz', euler, degrees=True)
        quaternion = r.as_quat()
        return quaternion
    
    # 四元数转欧拉角
    # ================OKOK
    def quaternion_to_euler(self,q, degree_mode=1):
        qw, qx, qy, qz = q

        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
        pitch = math.asin(2 * (qw * qy - qz * qx))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
        # degree_mode=1:【输出】是角度制，否则弧度制
        if degree_mode == 1:
            roll = np.rad2deg(roll)
            pitch = np.rad2deg(pitch)
            yaw = np.rad2deg(yaw)
        euler = np.array([roll, pitch, yaw])
        return euler
 
    # 欧拉角转四元数s
    # ================OKOK
    def euler_to_quaternion(self,euler, degree_mode=0):
        roll, pitch, yaw = euler
        # degree_mode=1:【输入】是角度制，否则弧度制
        if degree_mode == 1:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q = np.array([qw, qx, qy, qz])
        return q

    
    def H_(self,data):
        re = ord(chr(data)) * 256
        if re > 32767:
            return re - 65536
        else:
            return re
        
    def L_(self,data):
        re = ord(chr(data))
        return re
    
    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":    
            self.data = self.IMU.read_all()
            if len(self.data) > 0:
                if 0xa5 == self.data[0] and 0x5a == self.data[1] and 0xaa == self.data[len(self.data)-1]:
                    yaw_deg = ( self.H_(self.data[3]) + self.L_(self.data[4]) )/10.0
                    # yaw_deg = self.imu_yaw_calibration
                    if yaw_deg > 180.0:
                        yaw_deg = yaw_deg - 360.0
                    if yaw_deg < -180.0:
                        yaw_deg = yaw_deg + 360.0

                    self.yaw    = self.degrees2rad * yaw_deg
                    self.pitch  = self.degrees2rad * ( self.H_(self.data[7]) + self.L_(self.data[8]) )/10.0
                    self.roll   = self.degrees2rad * ( self.H_(self.data[5]) + self.L_(self.data[6]) )/10.0
                    self.accel_data['x'] = 9.806 * ( self.H_(self.data[9]) + self.L_(self.data[10]) ) / 16384.0
                    self.accel_data['y'] = 9.806 * ( self.H_(self.data[11]) + self.L_(self.data[12]) ) / 16384.0
                    self.accel_data['z'] = 9.806 * ( self.H_(self.data[13]) + self.L_(self.data[14]) ) / 16384.0

                    self.gyro_data['x'] = self.degrees2rad * ( self.H_(self.data[15]) + self.L_(self.data[16]) ) / 32.8
                    self.gyro_data['y'] = self.degrees2rad * ( self.H_(self.data[17]) + self.L_(self.data[18]) ) / 32.8
                    self.gyro_data['z'] = self.degrees2rad * ( self.H_(self.data[19]) + self.L_(self.data[20]) ) / 32.8
                    self.euler[0] = self.roll
                    self.euler[1] = self.pitch
                    self.euler[2] = self.yaw
                    print(self.euler)
                    #[self.yaw,self.pitch,self.roll]
                    q = self.euler_to_quaternion(self.euler)
                    self.orientation['x']=q[0]
                    self.orientation['y']=q[1]
                    self.orientation['z']=q[2]
                    self.orientation['w']=q[3]
                    # print(pa.array([direction]))
                    imu_dict = {
                        "header":{
                            "frame_id": "map"
                        },
                        "orientation":{
                            "x": self.orientation['x'],
                            "y": self.orientation['y'],
                            "z": self.orientation['z'],
                            "w": self.orientation['w'],
                        },
                        "linear_acceleration": {
                            "x": self.gyro_data['x'],
                            "y": self.gyro_data['y'],
                            "z": self.gyro_data['z'],
                        },
                        "angular_velocity": {
                            "x": self.accel_data['x'],
                            "y": self.accel_data['y'],
                            "z": self.accel_data['z'],
                        },
                    }
                    # 假设 json_string 是收到的 JSON 数据字符串
                    # json_dict = json.loads(imu_dict)
                    # 将 imu_dict 转换为 JSON 格式的字符串
                    # json_string = json.dumps(imu_dict)
                    # 将 JSON 字符串转换为字节串
                    json_string = json.dumps(imu_dict, indent=4)  # 使用indent参数设置缩进宽度为4
                    print(json_string)
                    json_bytes = json_string.encode('utf-8')
                    print(pa.array([imu_dict]))
                    self.imu_writer.publish(pa.array([imu_dict]))
                    # serialized_data = pickle.dumps(imu_dict)
                    # send_output("imu100D4",serialized_data,dora_event["metadata"])
                    send_output("imu100D4",json_bytes,dora_event["metadata"])
                    self.IMU.flushInput() 


        
        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
