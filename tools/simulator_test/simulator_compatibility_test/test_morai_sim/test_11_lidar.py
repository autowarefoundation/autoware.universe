from test_base.test_11_lidar import Test11LidarBase


class Test11LidarMorai(Test11LidarBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_lidars(self):
        result = self.update_pointcloud_data()
        for sensor in self.sensors:
            assert len(result[sensor["topic"]]) != 0
