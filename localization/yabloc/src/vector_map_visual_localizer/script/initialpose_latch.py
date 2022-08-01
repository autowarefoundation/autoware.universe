#!/usr/bin/python3
from rclpy.time import Time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class InitialPoseLatch(Node):
    def __init__(self):
        super().__init__('initial_pose_latch')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_ALL)
        self.sub_clock_ = self.create_subscription(
            Clock, '/clock',  self.clock_callback, qos)
        self.sub_pose_ = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',  self.pose_callback, 10)

        self.pub_pose_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.last_clock = None
        self.last_initial_psoe_and_time = None
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.last_clock is None:
            return
        if self.last_initial_psoe_and_time is None:
            return

        target = self.last_initial_psoe_and_time[0]
        dt = (self.last_clock.nanoseconds-target.nanoseconds)/1e9
        print('diff is', dt)
        if(abs(dt) < 1):
            self.pub_pose_.publish(self.last_initial_psoe_and_time[1])

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp
        stamp = Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
        self.get_logger().info('Subscribed initialpose: "%s"' + str(stamp))
        self.last_initial_psoe_and_time = (stamp, msg)

    def clock_callback(self, msg: Clock):
        # self.get_logger().info('now time is %s'+str(msg))
        self.last_clock = Time(seconds=msg.clock.sec, nanoseconds=msg.clock.nanosec)


def main(args=None):
    rclpy.init(args=args)

    initialpose_latch = InitialPoseLatch()
    rclpy.spin(initialpose_latch)
    initialpose_latch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
