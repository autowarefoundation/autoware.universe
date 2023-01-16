from test_base.test_10_imu_sensor import Test10ImuSensorBase


class Test10ImuSensorMorai(Test10ImuSensorBase):
    def test_1_imu(self, setup_method):
        self.update_sensor_data()
        for sensor in self.sensors:
            result = self.get_data(sensor["topic"])
            assert len(result) != 0
