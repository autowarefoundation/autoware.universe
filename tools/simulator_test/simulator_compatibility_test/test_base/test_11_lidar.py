import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from test_base.configuration_loader import ConfigFileHandler


class Test11LidarBase:
    node = None
    msgs_rx = {}
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        cls.configHandler = ConfigFileHandler("sensor_configurations.json")
        cls.configHandler.load()
        cls.sensors = cls.configHandler.get_lidar_list()

        QOS_BEKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        rclpy.init()
        cls.node = rclpy.create_node("test_11_lidar_base")
        for sensor in cls.sensors:
            cls.msgs_rx[sensor["topic"]] = []
            cls.node.create_subscription(
                PointCloud2,
                sensor["topic"],
                lambda msg: cls.msgs_rx[sensor["topic"]].append(msg),
                QOS_BEKL10V,
            )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        for sensor in self.sensors:
            self.sensor_msgs_rx[sensor["topic"]].clear()

    def update_pointcloud_data(self):
        time.sleep(1)
        cnt = 0
        try:
            if rclpy.ok():
                while cnt < 2:
                    self.executor.spin_once()
                    time.sleep(1)
                    cnt += 1
        except BaseException as e:
            print(e)
        finally:
            return self.msgs_rx

    def get_data(self, topic):
        return self.msgs_rx[topic]
