import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu


class Test10ImuSensorBase:
    sensor_msgs_rx = []
    node = None
    sub_imu = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_10_imu_sensor_base")
        cls.sub_pose_stamped = cls.node.create_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",
            lambda msg: cls.sensor_msgs_rx.append(msg),
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
