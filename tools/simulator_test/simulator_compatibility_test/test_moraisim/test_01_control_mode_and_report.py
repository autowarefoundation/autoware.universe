import os
import sys
import time
import pytest

import rclpy
from rclpy.executors import MultiThreadedExecutor

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from test_base.test_01_control_mode_and_report import Test01ControlModeAndReportBase
from simulator_compatibility_test.publishers.control_mode_command import ControlModeCommand_Constants
from simulator_compatibility_test.subscribers.control_mode_report import ControlModeReport_Constants
from simulator_compatibility_test.subscribers.control_mode_report import SubscriberControlModeReport


class Test01ControlModeAndReportMorai(Test01ControlModeAndReportBase):
    
    def test_1_manual_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.MANUAL)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.MANUAL.value
    
    def test_2_auto_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.AUTONOMOUS)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.AUTONOMOUS.value
        