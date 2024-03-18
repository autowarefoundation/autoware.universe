from typing import Callable
from dora import DoraStatus
import serial
from typing import Callable
from dora import DoraStatus
import math
import pyarrow as pa
import dora
import random
serial_port = "/dev/ttyUSB0"
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
        self.accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
        self.degrees2rad = math.pi/180.0
        self.header = {'seq':0.0,'frame_id': 'map', 'stamp': 0.0,}
        self.accel_data = {'x': random.random(), 'y': random.random(), 'z': random.random()}
        self.gyro_data = {'x': random.random(), 'y': random.random(), 'z': random.random()}
        self.orientation = {'x': 1.10, 'y': 1.20, 'z': 1.30,'w':1.04}
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
        """
        self.IMU = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1)
        if self.IMU and self.IMU.is_open:
            self.IMU.close()
        self.IMU.open()
        self.tmp = self.IMU.write(('\xA5\x5A\x04\x01\x05\xAA' + chr(13)).encode())

        # try:
        #     self.IMU = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1)
        #     if self.IMU and self.IMU.is_open:
        #         self.IMU.close()
        #     self.IMU.open()
        #     self.tmp = self.IMU.write(('\xA5\x5A\x04\x01\x05\xAA' + chr(13)).encode())

        # except serial.SerialException as ex:
        #     print(
        #         "Could not open serial port: I/O error({0}): {1}".format(
        #             ex.errno, ex.strerror
        #         )
        #     )
        # # finally:
        # #     if self.IMU and self.IMU.is_open:
        # #         self.IMU.close()
        # pass

    
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
                    yaw_deg = self.imu_yaw_calibration
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
  
                   
                    imu_dict = {
                        
                        "angular_velocity": {
                            "x": self.accel_data['x'],
                            "y": self.accel_data['y'],
                            "z": self.accel_data['z'],
                        },
                        "linear_acceleration": {
                            "x": self.gyro_data['x'],
                            "y": self.gyro_data['y'],
                            "z": self.gyro_data['z'],
                        },
                    }
                    print(imu_dict)
                    self.imu_writer.publish(pa.array([imu_dict]))
        
        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
    
