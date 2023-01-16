import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from test_base.configuration_loader import ConfigFileHandler


class Test08CameraSensorBase:
    sensor_msgs_rx = {}
    sensor_info_msgs_rx = {}
    node = None
    sub_sensor = None
    sub_sensor_info = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        cls.configHandler = ConfigFileHandler("sensor_configurations.json")
        cls.configHandler.load()
        cls.sensors = cls.configHandler.get_camera_list()

        QOS_BEKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        rclpy.init()
        cls.node = rclpy.create_node("test_08_camera_sensor_base")
        for sensor in cls.sensors:
            if "Image" in sensor["msg"]:
                cls.sensor_msgs_rx[sensor["topic"]] = []
                cls.sub_sensor = cls.node.create_subscription(
                    Image,
                    sensor["topic"],
                    lambda msg: cls.sensor_msgs_rx[sensor["topic"]].append(msg),
                    QOS_BEKL10V,
                )
            elif "CameraInfo" in sensor["msg"]:
                cls.sensor_info_msgs_rx[sensor["topic"]] = []
                cls.sub_sensor_info = cls.node.create_subscription(
                    CameraInfo,
                    sensor["topic"],
                    lambda msg: cls.sensor_info_msgs_rx[sensor["topic"]].append(msg),
                    QOS_BEKL10V,
                )
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        for sensor in self.sensors:
            if "Image" in sensor["msg"]:
                self.sensor_msgs_rx[sensor["topic"]].clear()
            elif "CameraInfo" in sensor["msg"]:
                self.sensor_info_msgs_rx[sensor["topic"]].clear()

    def update_sensor_info(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.sensor_info_msgs_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.sensor_info_msgs_rx

    def update_sensor_data(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.sensor_msgs_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.sensor_msgs_rx

    def get_data(self, topic):
        if topic in self.sensor_msgs_rx:
            return self.sensor_msgs_rx[topic]
        elif topic in self.sensor_info_msgs_rx:
            return self.sensor_info_msgs_rx[topic]
        else:
            return []
