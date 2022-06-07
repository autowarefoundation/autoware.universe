import os
import sys
import time
import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from autoware_auto_vehicle_msgs.msg import ControlModeCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport

from grpc_ros2_bridge_test_automation.publishers.control_mode_command import ControlModeCommand_Constants
from grpc_ros2_bridge_test_automation.subscribers.control_mode_report import ControlModeReport_Constants
from grpc_ros2_bridge_test_automation.subscribers.control_mode_report import SubscriberControlModeReport


class TestControlModeCommandReport:

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.sub_control_mode_report = SubscriberControlModeReport()
        cls.node = rclpy.create_node('test_control_mode_command_report')
        cls.sub = cls.node.create_subscription(
            ControlModeCommand,
            '/control/command/control_mode_cmd',
            lambda msg : cls.msgs_rx.append(msg),
            10
        )
        cls.pub = cls.node.create_publisher(
            ControlModeCommand,
            '/control/command/control_mode_cmd',
            10
        )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_control_mode_report)
        cls.executor.add_node(cls.node)
    
    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()
    
    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()
    
    def set_control_mode(self, control_mode):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_control_mode_cmd_msg(control_mode))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.mode == control_mode.value
        self.msgs_rx.clear()
    
    def generate_control_mode_cmd_msg(self, control_mode):
        stamp = self.node.get_clock().now().to_msg()
        msg = ControlModeCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.mode = control_mode.value
        return msg
    
    def get_control_mode_report(self):
        time.sleep(1)
        self.sub_control_mode_report.received.clear()
        received = ControlModeReport_Constants.NO_COMMAND
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_control_mode_report.received) > 2:
                    break
            received = self.sub_control_mode_report.received.pop()
        finally:
            return received
    
    def test_1_manual_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.MANUAL)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.MANUAL.value
    
    def test_2_auto_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.AUTONOMOUS)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.AUTONOMOUS.value
        

