from test_base.test_08_camera_sensor import Test08CameraSensorBase


class Test08CameraSensorMorai(Test08CameraSensorBase):
    def test_1_camera_info(self, setup_method):
        self.update_sensor_info()
        for sensor in self.sensors:
            result = self.get_data(sensor["topic"])
            assert len(result) != 0

    def test_2_camera_sensor(self, setup_method):
        self.update_sensor_data()
        for sensor in self.sensors:
            result = self.get_data(sensor["topic"])
            assert len(result) != 0
