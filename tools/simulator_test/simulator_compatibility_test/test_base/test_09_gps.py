import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from test_base.configuration_loader import ConfigFileHandler


class Test09GpsBase:
    rx_msgs = {}
    node = None
    sub_pose_stamped = None
    sub_pose_cov_stemped = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        cls.configHandler = ConfigFileHandler("sensor_configurations.json")
        cls.configHandler.load()
        cls.sensors = cls.configHandler.get_gps_list()

        rclpy.init()
        cls.node = rclpy.create_node("test_09_gps_base")
        for sensor in cls.sensors:
            if "PoseStamped" in sensor["msg"]:
                cls.rx_msgs[sensor["topic"]] = []
                pose_cb = cls.rx_closure(sensor["topic"])
                cls.sub_pose_stamped = cls.node.create_subscription(
                    PoseStamped,
                    sensor["topic"],
                    pose_cb,
                    10,
                )
            elif "PoseWithCovarianceStamped" in sensor["msg"]:
                cls.rx_msgs[sensor["topic"]] = []
                pose_cov_cb = cls.rx_closure(sensor["topic"])
                cls.sub_pose_cov_stemped = cls.node.create_subscription(
                    PoseWithCovarianceStamped,
                    sensor["topic"],
                    pose_cov_cb,
                    10,
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
