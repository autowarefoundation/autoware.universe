from test_base.test_09_gps import Test09GpsBase


class Test09GpsMorai(Test09GpsBase):
    def test_1_pose(self, setup_method):
        result = self.update_sensor_data()
        for sensor in self.sensors:
            if "PoseStamped" in sensor["msg"]:
                assert len(result[sensor["topic"]]) != 0

    def test_2_pose_with_covariance(self, setup_method):
        result = self.update_sensor_data()
        for sensor in self.sensors:
            if "PoseWithCovarianceStamped" in sensor["msg"]:
                assert len(result[sensor["topic"]]) != 0
