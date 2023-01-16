from test_base.test_11_lidar import Test11LidarBase


class Test11LidarSim(Test11LidarBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_get_left_raw(self):
        self.update_pointcloud_data()
        for sensor in self.sensors:
            result = self.get_data(sensor["topic"])
            assert len(result) != 0
