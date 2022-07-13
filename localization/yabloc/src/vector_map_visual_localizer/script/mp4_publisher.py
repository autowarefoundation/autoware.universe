#!/usr/bin/python3
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
import sys
import cv2
from cv_bridge import CvBridge


class Mp4Publisher(Node):
    def __init__(self, mp4_file):
        super().__init__('mp4_publisher')
        self.publisher_ = self.create_publisher(Image, '/sensing/camera/undistorted/image_raw', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.video_ = cv2.VideoCapture(mp4_file)
        self.bridge_ = CvBridge()

    def timer_callback(self):
        ret, frame = self.video_.read()

        msg = self.bridge_.cv2_to_imgmsg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/map"
        msg._encoding = 'bgr8'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing')


def main(args=None):
    if len(sys.argv) == 1:
        return

    rclpy.init(args=args)

    mp4_publisher = Mp4Publisher(sys.argv[1])
    rclpy.spin(mp4_publisher)
    mp4_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
