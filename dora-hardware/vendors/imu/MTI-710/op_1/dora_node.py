#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# The data sent by the Xsens sensor includes the two headers of Timestamp and Status, and there must be corresponding functions,I let the header find the corresponding data processing function by using the function find_handler_name(self, name), but after I optimized the original code from producer, this program no longer need to process the values of these two variables separately in a certain module. So these two functions are required in the form and these two pieces of code cannot be deleted.
import pickle
from typing import Callable
import datetime
import time
from dora import Node
import select
import sys
# from pynput import keyboard

import mtdevice
import mtdef

from math import radians, sqrt, atan2
# transform Euler angles or matrix into quaternions
# from TFTransformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix
from dora_assis import Imu, Header, TFTransformations, Parameter, My_parameter
import serial  # 是python serial 库是一个用于与串口通信的库。它提供了与串口设备进行通信的功能，允许ROS节点通过串口与外部硬件设备进行数据交换

from dora import DoraStatus


class Operator:
    """
    initial the xsnes_driver and its parameters
    """

    def __init__(self):
        device = '/dev/ttyUSB0'
        print(device)
        baudrate = 921600
        print(baudrate)
        timeout = 0.002
        initial_wait = 0.01
        if device == 'auto':
            devs = mtdevice.find_devices(timeout=timeout, initial_wait=initial_wait)
            if devs:
                device, baudrate = devs[0]
                # self.get_logger().info("Detected MT device on port %s @ %d bps" % (device, baudrate))
            else:
                # 没找到正确的device
                sys.exit(1)
        if not baudrate:
            baudrate = mtdevice.find_baudrate(device, timeout=timeout, initial_wait=initial_wait)
        if not baudrate:
            # self.get_logger().error("Fatal: could not find proper baudrate.")
            sys.exit(1)

        self.mt = mtdevice.MTDevice(device, baudrate, timeout, initial_wait=initial_wait)

        self.frame_id = 'base_imu'

        self.myparameter = My_parameter()
        parameter_value = self.myparameter.get_param("frame_local", 'ENU')
        self.frame_local = parameter_value
        print("this is frame_id")
        print(self.frame_local)

        self.angular_velocity_covariance = [radians(0.025)] * 3
        self.linear_acceleration_covariance = [0.0004] * 3
        self.orientation_covariance = [radians(1.), radians(1.), radians(9.)]

        self.last_delta_q_time = None
        self.delta_q_rate = None

    def reset_vars(self):
        self.imu_msg = Imu()
        self.imu_msg.orientation_covariance = (-1.,) * 9
        self.imu_msg.angular_velocity_covariance = (-1.,) * 9
        self.imu_msg.linear_acceleration_covariance = (-1.,) * 9
        self.pub_imu = False

    def find_handler_name(self, name):
        return "fill_from_%s" % (name.replace(" ", "_"))

    def fill_from_Acceleration(self, o):
        """Fill messages with information from 'Acceleration' MTData2 block."""
        self.pub_imu = True

        # FIXME not sure we should treat all in that same way
        try:
            x, y, z = o['Delta v.x'], o['Delta v.y'], o['Delta v.z']
        except KeyError:
            pass
        try:
            x, y, z = o['freeAccX'], o['freeAccY'], o['freeAccZ']
        except KeyError:
            pass
        try:
            x, y, z = o['accX'], o['accY'], o['accZ']
        except KeyError:
            pass
        self.imu_msg.linear_acceleration.x = x
        self.imu_msg.linear_acceleration.y = y
        self.imu_msg.linear_acceleration.z = z
        self.imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

    def fill_from_Angular_Velocity(self, o):
        """Fill messages with information from 'Angular Velocity' MTData2
        block."""
        try:
            dqw, dqx, dqy, dqz = (o['Delta q0'], o['Delta q1'],
                                  o['Delta q2'], o['Delta q3'])
            now = datetime.datetime.now()
            if self.last_delta_q_time is None:
                self.last_delta_q_time = now
            else:
                # update rate (filtering so as to account for lag variance（延迟方差）)
                # 算上次时间戳 self.last_delta_q_time 与当前时间 now 之间的时间差，单位为秒
                delta_t = (now - self.last_delta_q_time).microseconds * 1000 / 1e+9
                if self.delta_q_rate is None:
                    self.delta_q_rate = 1. / delta_t
                delta_t_filtered = .95 / self.delta_q_rate + .05 * delta_t
                # rate in necessarily integer
                self.delta_q_rate = round(1. / delta_t_filtered)
                self.last_delta_q_time = now
                # relationship between \Delta q and velocity \bm{\omega}:
                # \bm{w} = \Delta t . \bm{\omega}
                # \theta = |\bm{w}|
                # \Delta q = [cos{\theta/2}, sin{\theta/2)/\theta . \omega
                # extract rotation angle over delta_t
                ca_2, sa_2 = dqw, sqrt(dqx ** 2 + dqy ** 2 + dqz ** 2)
                # 计算用于旋转角度的余弦和正弦值。
                ca = ca_2 ** 2 - sa_2 ** 2
                sa = 2 * ca_2 * sa_2
                # 计算旋转角度
                rotation_angle = atan2(sa, ca)
                # compute rotation velocity
                rotation_speed = rotation_angle * self.delta_q_rate
                f = rotation_speed / sa_2
                x, y, z = f * dqx, f * dqy, f * dqz
                self.imu_msg.angular_velocity.x = x
                self.imu_msg.angular_velocity.y = y
                self.imu_msg.angular_velocity.z = z
                self.imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
                self.pub_imu = True
        except KeyError:
            pass
        try:
            x, y, z = o['gyrX'], o['gyrY'], o['gyrZ']
            self.imu_msg.angular_velocity.x = x
            self.imu_msg.angular_velocity.y = y
            self.imu_msg.angular_velocity.z = z
            self.imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
            self.pub_imu = True
        except KeyError:
            pass

    def fill_from_Timestamp(self, o):
        """Fill messages with information from 'Timestamp' MTData2 block.
        """
        pass

    def fill_from_Status(self, o):
        """Fill messages with information from 'Status' MTData2 block.
        """
        pass

    def fill_from_Orientation_Data(self, o):
        """Fill messages with information from 'Orientation Data' MTData2
        block."""
        self.pub_imu = True
        try:
            x, y, z, w = o['Q1'], o['Q2'], o['Q3'], o['Q0']
        except KeyError:
            pass
        try:
            x, y, z, w = TFTransformations.quaternion_from_euler(radians(o['Roll']),
                                                                 radians(o['Pitch']),
                                                                 radians(o['Yaw']))
        except KeyError:
            pass
        try:
            a, b, c, d, e, f, g, h, i = o['a'], o['b'], o['c'], o['d'], \
                                        o['e'], o['f'], o['g'], o['h'], o['i']
            m = TFTransformations.identity_matrix()
            m[:3, :3] = ((a, d, g),
                         (b, e, h),
                         (c, f, i))
            x, y, z, w = TFTransformations.quaternion_from_matrix(m)
        except KeyError:
            pass
        w, x, y, z = self.convert_quat((w, x, y, z), o['frame'])
        self.imu_msg.orientation.x = x
        self.imu_msg.orientation.y = y
        self.imu_msg.orientation.z = z
        self.imu_msg.orientation.w = w
        self.imu_msg.orientation_covariance = self.orientation_covariance

    def convert_quat(self, q, source):
        """Convert a quaternion between ENU, NED, and NWU."""

        def q_mult(q0, q1):
            """Quaternion multiplication."""
            w0, x0, y0, z0 = q0
            w1, x1, y1, z1 = q1
            w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
            x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
            y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
            z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
            return (w, x, y, z)

        q_enu_ned = (0, 1. / sqrt(2), 1. / sqrt(2), 0)
        q_enu_nwu = (1. / sqrt(2), 0, 0, -1. / sqrt(2))
        q_ned_nwu = (0, -1, 0, 0)
        q_ned_enu = (0, -1. / sqrt(2), -1. / sqrt(2), 0)
        q_nwu_enu = (1. / sqrt(2), 0, 0, 1. / sqrt(2))
        q_nwu_ned = (0, 1, 0, 0)
        if source == 'ENU':
            if self.frame_local == 'ENU':
                return q
            elif self.frame_local == 'NED':
                return q_mult(q_enu_ned, q)
            elif self.frame_local == 'NWU':
                return q_mult(q_enu_nwu, q)
        elif source == 'NED':
            if self.frame_local == 'ENU':
                return q_mult(q_ned_enu, q)
            elif self.frame_local == 'NED':
                return q
            elif self.frame_local == 'NWU':
                return q_mult(q_ned_nwu, q)
        elif source == 'NWU':
            if self.frame_local == 'ENU':
                return q_mult(q_nwu_enu, q)
            elif self.frame_local == 'NED':
                return q_mult(q_nwu_ned, q)
            elif self.frame_local == 'NWU':
                return q

    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle data from Imu
        Args:
            dora_input (dict): Dict containing the "id", "data", and "metadata"
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        if dora_event["type"] == "INPUT":
            while True:
                self.tf_transfomations = TFTransformations()
                # 当同时按下"Ctrl"和"Q"键时，循环会被打破（break），结束程序的执行。
                # listener = keyboard.Listener(on_press=on_press)
                # listener.start()
                # listener.join()
                # get data
                try:
                    data = self.mt.read_measurement()
                except mtdef.MTTimeoutException:
                    time.sleep(0.1)
                    return

                # common header
                # 用于存储消息的通用头信息
                self.h = Header()
                # 将当前时间戳设置为 self.h 的时间戳。get_clock().now() 获取当前时间，然后使用.strftime("%Y-%m-%d
                # %H:%M:%S.%f")方法将其转换为dora消息时间戳格式。
                current_time_seconds = int(datetime.datetime.now().timestamp())
                # Step 2: Get the current microseconds
                microseconds = datetime.datetime.now().microsecond
                # Step 3: Format the message
                message = f"sec: {current_time_seconds}"
                message += f"nanosec: {microseconds * 1000}"
                self.h.stamp = message
                # 将帧标识符 self.frame_id 分配给 self.h.frame_id，指定消息的参考坐标系。
                self.h.frame_id = self.frame_id

                # set default values
                self.reset_vars()

                # fill messages based on available data fields
                # for n, o in data.items():
                #     globals()[find_handler_name(n)](o)
                for n, o in data.items():
                    if len(data) == 4:  # 添加條件判斷，如果data的數據長度等於4，沒有Orientation data，則跳過當前迭代，處理下一個項目
                        continue
                    elif len(data) == 5:  # 如果data的數據長度等於5，則執行處理代碼
                        handler_name = self.find_handler_name(n)
                        handler_function = getattr(self, handler_name, None)
                        if handler_function is not None:
                            handler_function(o)
                        else:
                            print(f"Handler function '{handler_name}' not found.")
                    else:
                        print("Invalid data length.")  # 處理其他數據長度情況，您可以根據需求進行調整
                # publish available information
                if self.pub_imu:
                    self.imu_msg.header = self.h
                    # 使用pickle将实例变量转换为字节形式
                    # 将属性整理为字典
                    imu_dict = {
                        "header": {
                            "frame_id": self.imu_msg.header.frame_id,
                            "stamp": self.imu_msg.header.stamp
                        },
                        "orientation": {
                            "x": self.imu_msg.orientation.x,
                            "y": self.imu_msg.orientation.y,
                            "z": self.imu_msg.orientation.z,
                            "w": self.imu_msg.orientation.w,
                        },
                        "orientation_covariance": self.imu_msg.orientation_covariance,
                        "angular_velocity": {
                            "x": self.imu_msg.angular_velocity.x,
                            "y": self.imu_msg.angular_velocity.y,
                            "z": self.imu_msg.angular_velocity.z,
                        },
                        "angular_velocity_covariance": self.imu_msg.angular_velocity_covariance,
                        "linear_acceleration": {
                            "x": self.imu_msg.linear_acceleration.x,
                            "y": self.imu_msg.linear_acceleration.y,
                            "z": self.imu_msg.linear_acceleration.z,
                        },
                        "linear_acceleration_covariance": self.imu_msg.linear_acceleration_covariance,
                    }
                    # 将 imu_dict 转换为字节类型
                    serialized_data = pickle.dumps(imu_dict)
                    send_output("imu-data",
                                serialized_data,
                                dora_event["metadata"])
                    # reset default values
                    self.reset_vars()

            return DoraStatus.CONTINUE

# ---For test---

# def mock_send_output(output_type: str, data: bytes):
#     print(f"Sending output of type '{output_type}': {data}")
#
#
# def main(args=None):
#     """Create a  node and instantiate the class."""
#
#     driver = Operator()
#     send_output_input = mock_send_output
#     dora_event_input = {
#         "type": "INPUT",
#     }
#     driver.on_event(dora_event_input, send_output_input)
#
#
# if __name__ == '__main__':
#     main()
