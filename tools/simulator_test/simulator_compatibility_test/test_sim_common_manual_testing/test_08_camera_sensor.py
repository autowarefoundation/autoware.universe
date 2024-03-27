from test_base.test_08_camera_sensor import Test08CameraSensorBase


class Test08CameraSensorSim(Test08CameraSensorBase):
    def test_1_camera_info(self, setup_method):
        result = self.update_sensor_data()
        for sensor in self.sensors:
            if "CameraInfo" in sensor["msg"]:
                assert len(result[sensor["topic"]]) != 0

    def test_2_camera_sensor(self, setup_method):
        result = self.update_sensor_data()
        for sensor in self.sensors:
            if "Image" in sensor["msg"]:
                assert len(result[sensor["topic"]]) != 0
