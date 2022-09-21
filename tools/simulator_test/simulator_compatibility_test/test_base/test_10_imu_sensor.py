import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu


class Test10ImuSensorBase:
    sensor_msgs_rx = []
    node = None
    sub_imu = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        QOS_BEKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        rclpy.init()
        cls.node = rclpy.create_node("test_10_imu_sensor_base")
        cls.sub_pose_stamped = cls.node.create_subscription(
            Imu,
            "/sensing/imu/tamagawa/imu_raw",
            lambda msg: cls.sensor_msgs_rx.append(msg),
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
