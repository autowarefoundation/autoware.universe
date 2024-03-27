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
    rx_msgs = {}
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
            cls.rx_msgs[sensor["topic"]] = []
            lidar_cb = cls.rx_closure(sensor["topic"])
            cls.node.create_subscription(
                PointCloud2,
                sensor["topic"],
                lidar_cb,
                QOS_BEKL10V,
            )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def rx_closure(cls, topic):
        def rx_callback(msg):
            cls.rx_msgs[topic].append(msg)

        return rx_callback

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        for sensor in self.sensors:
            self.rx_msgs[sensor["topic"]].clear()

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
            return self.rx_msgs
