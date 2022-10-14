#!/usr/bin/python3
from rclpy.time import Time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter


class InitialPoseLatch(Node):
    def __init__(self):
        super().__init__('initial_pose_latch')

        self.sub_pose_ = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',  self.pose_callback, 10)

        self.pub_pose_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.last_initial_psoe_and_time = None
        self.timer_ = self.create_timer(1.0, self.timer_callback, clock=self.get_clock())
        self.last_time_diff = None

    def timer_callback(self):
        if self.last_initial_psoe_and_time is None:
            return

        now_stamp = self.get_clock().now()
        target = self.last_initial_psoe_and_time[0]
        dt = abs(now_stamp.nanoseconds-target.nanoseconds)/1e9

        if self.last_time_diff is not None:
            if dt < self.last_time_diff:
                self.get_logger().info('It will publish initialpose soon ' + str(dt))

        if dt < 0.55:
            self.get_logger().info('Publish initialpose')
            self.pub_pose_.publish(self.last_initial_psoe_and_time[1])

        self.last_time_diff = dt

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        stamp = msg.header.stamp
        stamp = Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
        self.get_logger().info('Subscribed initialpose')
        self.last_initial_psoe_and_time = (stamp, msg)


def main(args=None):
    rclpy.init(args=args)

    initialpose_latch = InitialPoseLatch()
    rclpy.spin(initialpose_latch)
    initialpose_latch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
