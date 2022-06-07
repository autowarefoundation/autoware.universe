import os
import sys
import time
from math import pi
import pytest

import rclpy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_control_msgs.msg import AckermannLateralCommand
from autoware_auto_control_msgs.msg import LongitudinalCommand

from grpc_ros2_bridge_test_automation.clients.client_event_cmd import ClientEventCmdAsync
from grpc_ros2_bridge_test_automation.publishers.morai_ctrl_cmd import LongCmdType
from grpc_ros2_bridge_test_automation.publishers.morai_ctrl_cmd import PublisherMoraiCtrlCmd
from grpc_ros2_bridge_test_automation.publishers.ackermann_control_command import PublisherAckermannControlCommand
from grpc_ros2_bridge_test_automation.subscribers.steering_report import SubscriberSteeringReport


class TestAckermannCtrlSteeringReport:

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.control_cmd = {
            'lateral': {
                'steering_tire_angle': 0.0,
                'steering_tire_rotation_rate': 0.0
            },
            'longitudinal': {
                'speed': 0.0,
                'acceleration': 0.0,
                'jerk': 0.0
            }
        }
        
        cls.node = rclpy.create_node('test_ackermann_control_steering_report')
        cls.sub = cls.node.create_subscription(
            AckermannControlCommand,
            '/control/command/control_cmd',
            lambda msg : cls.msgs_rx.append(msg),
            10
        )
        cls.pub = cls.node.create_publisher(
            AckermannControlCommand,
            '/control/command/control_cmd',
            10
        )
        cls.sub_steering_report = SubscriberSteeringReport()
        cls.set_vehicle_auto_mode()
    
    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def set_vehicle_auto_mode(cls):
        event_cmd_client = ClientEventCmdAsync()
        event_cmd_client.send_request()

        while rclpy.ok():
            rclpy.spin_once(event_cmd_client)
            if event_cmd_client.future.done():
                result_msg = event_cmd_client.future.result()
            break
    
    @pytest.fixture
    def setup_and_teardown(self):
        self.control_cmd['lateral']['steering_tire_angle'] = 0.0
        self.control_cmd['longitudinal']['speed'] = 0.0
        self.set_morai_ctrl_mode(LongCmdType.VELOCITY)
        yield time.sleep(3)
        self.init_vehicle()
        time.sleep(3)    
        
    def init_vehicle(self):
        self.set_steering_tire_angle(0.0)
    
    def set_morai_ctrl_mode(self, long_cmd_type):
        ctrl_cmd_publisher = PublisherMoraiCtrlCmd()
        
        msg = {'longCmdType': long_cmd_type.value,
               'accel': 0.0,
               'brake': 0.0,
               'steering': 0.0,
               'velocity': 0.0,
               'acceleration': 0.0}
        ctrl_cmd_publisher.publish_msg(msg)
    
    def generate_control_msg(self, control_cmd):
        stamp = self.node.get_clock().now().to_msg()
        msg = AckermannControlCommand()
        lateral_cmd = AckermannLateralCommand()
        longitudinal_cmd = LongitudinalCommand()
        lateral_cmd.stamp.sec = stamp.sec
        lateral_cmd.stamp.nanosec = stamp.nanosec
        lateral_cmd.steering_tire_angle = control_cmd['lateral']['steering_tire_angle']
        lateral_cmd.steering_tire_rotation_rate = control_cmd['lateral']['steering_tire_rotation_rate']
        longitudinal_cmd.stamp.sec = stamp.sec
        longitudinal_cmd.stamp.nanosec = stamp.nanosec
        longitudinal_cmd.speed = control_cmd['longitudinal']['speed']
        longitudinal_cmd.acceleration = control_cmd['longitudinal']['acceleration']
        longitudinal_cmd.jerk = control_cmd['longitudinal']['jerk']
        
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.lateral = lateral_cmd
        msg.longitudinal = longitudinal_cmd
        return msg
        
    
    def set_steering_tire_angle(self, angle_rad):
        self.set_morai_ctrl_mode(LongCmdType.VELOCITY)
        self.control_cmd['lateral']['steering_tire_angle'] = angle_rad
        self.msgs_rx.clear()
        while rclpy.ok():
            rclpy.spin_once(self.node)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  round(received.lateral.steering_tire_angle, 2) == round(angle_rad, 2)
        self.msgs_rx.clear()
    
    def get_steering_report(self):
        self.sub_steering_report.received.clear()
        received = 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(self.sub_steering_report)
                if len(self.sub_steering_report.received) > 2:
                    break
            received = self.sub_steering_report.received.pop()
        finally:
            return received
    
    def test_1_grater_than_zero(self, setup_and_teardown):
        target_value = pi / 6
        self.set_steering_tire_angle(target_value)
        time.sleep(3)
        current_report = self.get_steering_report()
        assert current_report.steering_tire_angle > 0.0
    
    def test_2_less_than_zero(self, setup_and_teardown):
        target_value = (-1) * pi / 6
        self.set_steering_tire_angle(target_value)
        time.sleep(3)
        current_report = self.get_steering_report()
        assert current_report.steering_tire_angle < 0.0
        
        