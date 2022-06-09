import os
import sys
import rclpy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from test_base.test_02_change_gear_and_report import Test02ChangeGearAndReportBase
from test_base.test_02_change_gear_and_report import GearMode
from simulator_compatibility_test.clients.moraisim.morai_client_event_cmd import ClientEventCmdAsync
from simulator_compatibility_test.subscribers.gear_report import SubscriberGearReport


class Test02ChangeGearAndReportMorai(Test02ChangeGearAndReportBase):
    
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
