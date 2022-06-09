import time
import pytest
from enum import Enum
import rclpy
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import GearReport
from simulator_compatibility_test.subscribers.gear_report import GearMode
from simulator_compatibility_test.subscribers.gear_report import SubscriberGearReport


class Test02ChangeGearAndReportBase:
    msgs_rx = []
    node = None
    sub = None
    pub = None
    sub_gear_report = None

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.node = rclpy.create_node('test_02_change_gear_and_report_base')
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
    
    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()
    
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
    