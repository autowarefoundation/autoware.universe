from test_base.test_11_lidar import Test11LidarBase


class Test11LidarSim(Test11LidarBase):
    def test_get_left_raw(self):
        result = self.get_left_raw()
        assert len(result) != 0

    def test_get_left_raw_ex(self):
        result = self.get_left_raw_ex()
        assert len(result) != 0

    def test_get_right_raw(self):
        result = self.get_right_raw()
        assert len(result) != 0

    def test_get_right_raw_ex(self):
        result = self.get_right_raw_ex()
        assert len(result) != 0

    def test_get_top_raw(self):
        result = self.get_top_raw()
        assert len(result) != 0

    def test_get_top_raw_ex(self):
        result = self.get_top_raw_ex()
        assert len(result) != 0
