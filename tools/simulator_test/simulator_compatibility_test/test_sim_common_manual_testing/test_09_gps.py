from test_base.test_09_gps import Test09GpsBase


class Test09GpsSim(Test09GpsBase):
    def test_1_pose(self, setup_method):
        result = self.get_pose()
        assert len(result) != 0

    def test_2_pose_with_covariance(self, setup_method):
        result = self.get_pose_with_covariance()
        assert len(result) != 0
