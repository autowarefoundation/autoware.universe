import os
import sys
import time
import pytest
from enum import Enum

import rclpy
from rclpy.executors import MultiThreadedExecutor

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from autoware_auto_vehicle_msgs.msg import HazardLightsReport

from grpc_ros2_bridge_test_automation.subscribers.hazard_lights_report import HazardLightsReport_Constants
from grpc_ros2_bridge_test_automation.subscribers.hazard_lights_report import SubscriberHazardLightsReport


class HazardLightsCommand_Constants(Enum):
    NO_COMMAND = 0
    DISABLE = 1
    ENABLE = 2


class TestHazardLightsCommandReport:

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.sub_hazard_lights_status = SubscriberHazardLightsReport()
        cls.node = rclpy.create_node('test_hazard_lights_cmd_report')
        cls.sub = cls.node.create_subscription(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            lambda msg : cls.msgs_rx.append(msg),
            10
        )
        cls.pub = cls.node.create_publisher(
            HazardLightsCommand,
            '/control/command/hazard_lights_cmd',
            10
        )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_hazard_lights_status)
        cls.executor.add_node(cls.node)
    
    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()
    
    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()
    
    def set_hazard_lights(self, command):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_hazard_lights_cmd_msg(command))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.command == command.value
    
    def generate_hazard_lights_cmd_msg(self, command):
        stamp = self.node.get_clock().now().to_msg()
        msg = HazardLightsCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = command.value
        return msg
    
    def get_hazard_lights_status(self):
        time.sleep(1)
        self.sub_hazard_lights_status.received.clear()
        received = HazardLightsReport_Constants.DISABLE
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_hazard_lights_status.received) > 2:
                    break
            received = self.sub_hazard_lights_status.received.pop()
        finally:
            return received
    
    def test_1_enable(self, setup_method):
        hazard_lights = HazardLightsCommand_Constants.ENABLE
        self.set_hazard_lights(hazard_lights)
        result = self.get_hazard_lights_status()
        assert result.report == HazardLightsReport_Constants.ENABLE.value
    
    def test_2_disable(self, setup_method):
        hazard_lights = HazardLightsCommand_Constants.DISABLE
        self.set_hazard_lights(hazard_lights)
        result = self.get_hazard_lights_status()
        assert result.report == HazardLightsReport_Constants.DISABLE.value
        

