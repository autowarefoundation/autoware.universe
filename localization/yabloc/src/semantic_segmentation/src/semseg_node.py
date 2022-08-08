#!/usr/bin/env python3
import semseg_core
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge
import time


class SemsegNode(Node):
    def __init__(self):
        super().__init__('initial_pose_latch')

        model_path = self.declare_parameter('model_path', '').value
        self.get_logger().info('model path: ' + model_path)

        self.sub_image_ = self.create_subscription(
            Image, '/in/image_raw',  self.imageCallback, 10)

        self.pub_image_ = self.create_publisher(Image, '/out/image_raw', 10)
        self.dnn_ = semseg_core.SemSeg(model_path)
        self.bridge_ = CvBridge()

    def imageCallback(self, msg: Image):
        stamp = msg.header.stamp
        self.get_logger().info('Subscribed image: ' + str(stamp))

        image = self.bridge_.imgmsg_to_cv2(msg)
        start_time = time.time()
        mask = self.dnn_.inference(image)
        elapsed_time = time.time() - start_time

        show_image = self.dnn_.drawOverlayMask(image, mask)

        cv2.putText(show_image,
                    "Inference: " + '{:.1f}'.format(elapsed_time * 1000) + "ms",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                    cv2.LINE_AA)

        cv2.imshow('semseg',  show_image)
        cv2.waitKey(1)


def main():
    rclpy.init(args=sys.argv)

    semseg_node = SemsegNode()
    rclpy.spin(semseg_node)
    semseg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
