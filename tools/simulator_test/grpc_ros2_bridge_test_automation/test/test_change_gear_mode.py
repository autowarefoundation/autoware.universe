import os
import sys
import time
import pytest

import rclpy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import GearReport

from grpc_ros2_bridge_test_automation.clients.client_event_cmd import ClientEventCmdAsync
from grpc_ros2_bridge_test_automation.publishers.gear_command import GearMode
from grpc_ros2_bridge_test_automation.subscribers.gear_report import SubscriberGearReport


class TestGearCommandReport:

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.node = rclpy.create_node('test_gear_command_report')
        cls.sub = cls.node.create_subscription(
            GearCommand,
            '/control/command/gear_cmd',
            lambda msg : cls.msgs_rx.append(msg),
            10
        )
        cls.pub = cls.node.create_publisher(
            GearCommand,
            '/control/command/gear_cmd',
            10
        )
        cls.sub_gear_report = SubscriberGearReport()
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
    
    def generate_gear_msg(self, gear_mode):
        stamp = self.node.get_clock().now().to_msg()
        msg = GearCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = gear_mode.value
        return msg
    
    def set_gear_mode(self, gear_mode):
        self.msgs_rx.clear()
        while rclpy.ok():
            rclpy.spin_once(self.node)
            self.pub.publish(self.generate_gear_msg(gear_mode))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.command == gear_mode.value
        self.msgs_rx.clear()
    
    def get_gear_mode(self):
        time.sleep(1)
        self.sub_gear_report.received.clear()
        received = GearMode.NONE
        try:
            while rclpy.ok():
                rclpy.spin_once(self.sub_gear_report)
                if len(self.sub_gear_report.received) > 2:
                    break
            received = self.sub_gear_report.received.pop()
        finally:
            return received
    
    def test_1_gear_park(self):
        self.set_gear_mode(GearMode.PARK)
        result = self.get_gear_mode()
        assert result == GearMode.PARK.value
        
    def test_2_gear_neutral(self):
        self.set_gear_mode(GearMode.NEUTRAL)
        result = self.get_gear_mode()
        assert result == GearMode.NEUTRAL.value
        
    def test_3_gear_reverse(self):
        self.set_gear_mode(GearMode.REVERSE)
        result = self.get_gear_mode()
        assert result == GearMode.REVERSE.value
        
    def test_4_gear_drive(self):
        self.set_gear_mode(GearMode.DRIVE)
        result = self.get_gear_mode()
        assert result == GearMode.DRIVE.value

