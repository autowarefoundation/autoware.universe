import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from test_base.configuration_loader import ConfigFileHandler


class Test09GpsBase:
    pose_stamped_rx = {}
    pose_cov_stamped_rx = {}
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
                cls.pose_stamped_rx[sensor["topic"]] = []
                cls.sub_pose_stamped = cls.node.create_subscription(
                    PoseStamped,
                    sensor["topic"],
                    lambda msg: cls.pose_stamped_rx[sensor["topic"]].append(msg),
                    10,
                )
            if "PoseWithCovarianceStamped" in sensor["msg"]:
                cls.sub_pose_cov_stemped = cls.node.create_subscription(
                    PoseWithCovarianceStamped,
                    sensor["topic"],
                    lambda msg: cls.pose_cov_stamped_rx[sensor["topic"]].append(msg),
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
        for sensor in self.sensors:
            if "PoseStamped" in sensor["msg"]:
                self.pose_stamped_rx[sensor["topic"]].clear()
            elif "PoseWithCovarianceStamped" in sensor["msg"]:
                self.pose_cov_stamped_rx[sensor["topic"]].clear()

    def update_sensor_data(self):
        self.update_pose()
        self.update_pose_with_covariance()

    def update_pose(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.pose_stamped_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.pose_stamped_rx

    def update_pose_with_covariance(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.pose_cov_stamped_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.pose_cov_stamped_rx

    def get_data(self, topic):
        if topic in self.pose_stamped_rx:
            return self.pose_stamped_rx[topic]
        elif topic in self.pose_cov_stamped_rx:
            return self.pose_cov_stamped_rx[topic]
