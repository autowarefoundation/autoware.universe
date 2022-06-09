import os
import sys

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))

from test_base.test_06_hazard_lights_cmd_and_report import Test06HazardLightsCmdAndReportBase
from test_base.test_06_hazard_lights_cmd_and_report import HazardLightsCommand_Constants
from simulator_compatibility_test.subscribers.hazard_lights_report import HazardLightsReport_Constants
from simulator_compatibility_test.subscribers.hazard_lights_report import SubscriberHazardLightsReport


class Test06HazardLightsCmdAndReportMorai(Test06HazardLightsCmdAndReportBase):

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
        