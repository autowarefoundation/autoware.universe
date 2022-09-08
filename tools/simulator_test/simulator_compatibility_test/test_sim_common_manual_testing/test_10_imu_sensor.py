from test_base.test_10_imu_sensor import Test10ImuSensorBase


class Test10ImuSensorSim(Test10ImuSensorBase):
    def test_1_imu(self, setup_method):
        result = self.get_sensor_data()
        assert len(result) != 0
