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
    rx_msgs = {}
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
                cls.rx_msgs[sensor["topic"]] = []
                image_cb = cls.rx_closure(sensor["topic"])
                cls.sub_sensor = cls.node.create_subscription(
                    Image,
                    sensor["topic"],
                    image_cb,
                    QOS_BEKL10V,
                )
            elif "CameraInfo" in sensor["msg"]:
                cls.rx_msgs[sensor["topic"]] = []
                cam_info_cb = cls.rx_closure(sensor["topic"])
                cls.sub_sensor_info = cls.node.create_subscription(
                    CameraInfo,
                    sensor["topic"],
                    cam_info_cb,
                    QOS_BEKL10V,
                )
        cls.executor = SingleThreadedExecutor()
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

    def update_sensor_data(self):
        time.sleep(1)
        cnt = 0
        try:
            if rclpy.ok():
                while cnt <= 3:
                    self.executor.spin_once()
                    cnt += 1
        except BaseException as e:
            print(e)
        finally:
            return self.rx_msgs
