import os
import sys
import time
import pytest
import rclpy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from test_base.test_03_longitudinal_command_and_report import Test03LongitudinalCommandAndReportBase
from simulator_compatibility_test.clients.moraisim.morai_client_event_cmd import ClientEventCmdAsync
from simulator_compatibility_test.publishers.moraisim.morai_ctrl_cmd import PublisherMoraiCtrlCmd
from simulator_compatibility_test.publishers.moraisim.morai_ctrl_cmd import LongCmdType


class Test03LongitudinalCommandAndReportMorai(Test03LongitudinalCommandAndReportBase):

    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()
        cls.set_vehicle_auto_mode()
    
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
        self.control_cmd['longitudinal']['speed'] = 0.0
        self.control_cmd['longitudinal']['acceleration'] = 0.0
        self.set_morai_ctrl_mode(LongCmdType.VELOCITY)
        yield time.sleep(3)
        self.init_vehicle()
        time.sleep(3)    
    
    def set_morai_ctrl_mode(self, long_cmd_type):
        ctrl_cmd_publisher = PublisherMoraiCtrlCmd()
        
        msg = {'longCmdType': long_cmd_type.value,
               'accel': 0.0,
               'brake': 0.0,
               'steering': 0.0,
               'velocity': 0.0,
               'acceleration': 0.0}
        ctrl_cmd_publisher.publish_msg(msg)

    def set_speed(self, speed):
        self.set_morai_ctrl_mode(LongCmdType.VELOCITY)
        self.control_cmd['longitudinal']['speed'] = speed
        self.msgs_rx.clear()
        while rclpy.ok():
            rclpy.spin_once(self.node)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.longitudinal.speed == speed
        self.msgs_rx.clear()
    
    def set_acceleration(self, acceleration):
        self.set_morai_ctrl_mode(LongCmdType.ACCELERATION)
        self.control_cmd['longitudinal']['acceleration'] = acceleration
        self.msgs_rx.clear()
        while rclpy.ok():
            rclpy.spin_once(self.node)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2 :
                break
        received = self.msgs_rx[-1]
        assert  received.longitudinal.acceleration == acceleration
        self.msgs_rx.clear()

    def test_1_speed_control(self, setup_and_teardown):
        target_value = 100.0
        self.set_speed(target_value)
        time.sleep(3)
        current_speed = self.get_velocity_report()
        assert current_speed.longitudinal_velocity > 10.0
    
    def test_2_acceleration_control(self, setup_and_teardown):
        target_value = 100.0
        self.set_acceleration(target_value)
        time.sleep(3)
        current_speed = self.get_velocity_report()
        assert current_speed.longitudinal_velocity > 10.0
        