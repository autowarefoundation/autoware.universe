#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            depth=10
        )
        self.publisher_ = self.create_publisher(
            CompressedImage, '/sensing/camera/traffic_light/image_raw/compressed', qos)
        self.subscription = self.create_subscription(
            Image, '/sensing/camera/traffic_light/image_raw', self.on_image, qos)
        self.bridge_ = CvBridge()

    def on_image(self, msg):
        self.get_logger().info('subscribed')
        mat = self.bridge_.imgmsg_to_cv2(msg)
        compressed_msg = self.bridge_.cv2_to_compressed_imgmsg(mat)
        compressed_msg.header = msg.header
        self.publisher_.publish(compressed_msg)
        self.get_logger().info('published')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ImageCompressor()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
