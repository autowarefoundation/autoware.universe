from math import pi
import time

import pytest
import rclpy

from simulator_compatibility_test.clients.moraisim.morai_client_event_cmd \
    import ClientEventCmdAsync
from simulator_compatibility_test.publishers.moraisim.morai_ctrl_cmd \
    import LongCmdType
from simulator_compatibility_test.publishers.moraisim.morai_ctrl_cmd \
    import PublisherMoraiCtrlCmd
from test_base.test_04_lateral_command_and_report \
    import Test04LateralCommandAndReportBase


class Test04LateralCommandAndReportMorai(Test04LateralCommandAndReportBase):

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
                event_cmd_client.future.result()
            break

    @pytest.fixture
    def setup_and_teardown(self):
        self.control_cmd['lateral']['steering_tire_angle'] = 0.0
        self.control_cmd['longitudinal']['speed'] = 0.0
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

    def set_steering_tire_angle(self, angle_rad):
        self.set_morai_ctrl_mode(LongCmdType.VELOCITY)
        self.control_cmd['lateral']['steering_tire_angle'] = angle_rad
        self.msgs_rx.clear()
        while rclpy.ok():
            rclpy.spin_once(self.node)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert round(received.lateral.steering_tire_angle, 2) == \
            round(angle_rad, 2)
        self.msgs_rx.clear()

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
