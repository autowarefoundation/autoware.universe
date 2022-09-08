import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class Test08CameraSensorBase:
    sensor_msgs_rx = []
    sensor_info_msgs_rx = []
    node = None
    sub_sensor = None
    sub_sensor_info = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_08_camera_sensor_base")
        cls.sub_sensor = cls.node.create_subscription(
            Image,
            "/sensing/camera/traffic_light/image_raw",
            lambda msg: cls.sensor_msgs_rx.append(msg),
            10,
        )
        cls.sub_sensor_info = cls.node.create_subscription(
            CameraInfo,
            "/sensing/camera/traffic_light/camera_info",
            lambda msg: cls.sensor_info_msgs_rx.append(msg),
            10,
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
        self.sensor_msgs_rx.clear()
        self.sensor_info_msgs_rx.clear()

    def get_sensor_info(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.sensor_info_msgs_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.sensor_info_msgs_rx

    def get_sensor_data(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.sensor_msgs_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.sensor_msgs_rx
