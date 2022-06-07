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

from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsReport

from grpc_ros2_bridge_test_automation.subscribers.turn_indicators_report import TurnIndicatorsReport_Constants
from grpc_ros2_bridge_test_automation.subscribers.turn_indicators_report import SubscriberTurnIndicatorsReport


class TurnIndicatorsCommand_Constants(Enum):
    NO_COMMAND = 0
    DISABLE = 1
    ENABLE_LEFT = 2
    ENABLE_RIGHT = 3


class TestTurnIndicatorsCommandReport:

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.msgs_rx = []
        cls.sub_turn_indicators_status = SubscriberTurnIndicatorsReport()
        cls.node = rclpy.create_node('test_turn_indicators_report')
        cls.sub = cls.node.create_subscription(
            TurnIndicatorsCommand,
            '/control/command/turn_indicators_cmd',
            lambda msg : cls.msgs_rx.append(msg),
            10
        )
        cls.pub = cls.node.create_publisher(
            TurnIndicatorsCommand,
            '/control/command/turn_indicators_cmd',
            10
        )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_turn_indicators_status)
        cls.executor.add_node(cls.node)
    
    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()
    
    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()
    
    def set_turn_indicators(self, command):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_turn_indicators_cmd_msg(command))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.command == command.value
    
    def generate_turn_indicators_cmd_msg(self, command):
        stamp = self.node.get_clock().now().to_msg()
        msg = TurnIndicatorsCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = command.value
        return msg
    
    def get_turn_indicators_status(self):
        time.sleep(1)
        self.sub_turn_indicators_status.received.clear()
        received = TurnIndicatorsReport_Constants.DISABLE
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_turn_indicators_status.received) > 2:
                    break
            received = self.sub_turn_indicators_status.received.pop()
        finally:
            return received
    
    def test_1_enable_left(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.ENABLE_LEFT
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.ENABLE_LEFT.value
    
    def test_2_enable_right(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.ENABLE_RIGHT
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.ENABLE_RIGHT.value
        
    def test_3_disable(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.DISABLE
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.DISABLE.value

