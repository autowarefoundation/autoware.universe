import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


class Test09GpsBase:
    pose_stamped_rx = []
    pose_cov_stamped_rx = []
    node = None
    sub_pose_stamped = None
    sub_pose_cov_stemped = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_09_gps_base")
        cls.sub_pose_stamped = cls.node.create_subscription(
            PoseStamped,
            "/sensing/gnss/pose",
            lambda msg: cls.pose_stamped_rx.append(msg),
            10,
        )
        cls.sub_pose_cov_stemped = cls.node.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            lambda msg: cls.pose_cov_stamped_rx.append(msg),
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
        self.pose_stamped_rx.clear()
        self.pose_cov_stamped_rx.clear()

    def get_pose(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.pose_stamped_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.pose_stamped_rx

    def get_pose_with_covariance(self):
        time.sleep(1)
        try:
            if rclpy.ok():
                while len(self.pose_cov_stamped_rx) <= 2:
                    self.executor.spin_once()
        except BaseException as e:
            print(e)
        finally:
            return self.pose_cov_stamped_rx
