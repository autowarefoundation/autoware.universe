import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Test11LidarBase:
    node = None
    msgs_rx = {
        "left_raw_rx": [],
        "left_raw_ex_rx": [],
        "right_raw_rx": [],
        "right_raw_ex_rx": [],
        "top_raw_rx": [],
        "top_raw_ex_rx": [],
    }
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
        cls.node = rclpy.create_node("test_11_lidar_base")
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/left/pointcloud_raw",
            lambda msg: cls.msgs_rx["right_left_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/left/pointcloud_raw_ex",
            lambda msg: cls.msgs_rx["right_left_ex_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/right/pointcloud_raw",
            lambda msg: cls.msgs_rx["right_raw_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/right/pointcloud_raw_ex",
            lambda msg: cls.msgs_rx["right_raw_ex_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_raw",
            lambda msg: cls.msgs_rx["top_raw_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.node.create_subscription(
            PointCloud2,
            "/sensing/lidar/top/pointcloud_raw_ex",
            lambda msg: cls.msgs_rx["top_raw_ex_rx"].append(msg),
            QOS_BEKL10V,
        )
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

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

    def get_left_raw(self):
        return self.msgs_rx["left_raw_rx"]

    def get_left_raw_ex(self):
        return self.msgs_rx["left_raw_ex_rx"]

    def get_right_raw(self):
        return self.msgs_rx["right_raw_rx"]

    def get_right_raw_ex(self):
        return self.msgs_rx["right_raw_ex_rx"]

    def get_top_raw(self):
        return self.msgs_rx["top_raw_rx"]

    def get_top_raw_ex(self):
        return self.msgs_rx["top_raw_ex_rx"]
