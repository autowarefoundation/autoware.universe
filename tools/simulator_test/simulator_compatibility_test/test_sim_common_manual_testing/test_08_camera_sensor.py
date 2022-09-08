from test_base.test_08_camera_sensor import Test08CameraSensorBase


class Test08CameraSensorSim(Test08CameraSensorBase):
    def test_1_camera_info(self, setup_method):
        result = self.get_sensor_info()
        assert len(result) != 0

    def test_2_camera_sensor(self, setup_method):
        result = self.get_sensor_data()
        assert len(result) != 0
