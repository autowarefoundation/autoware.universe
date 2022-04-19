#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
# from lib.advanced_lane_finding.module import test


class Subscriber(Node):

    def __init__(self):
        super().__init__('lane_finding')
        self.sub_image= self.create_subscription(CompressedImage, '/sensing/camera/traffic_light/image_raw/compressed',self.callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/sensing/camera/traffic_light/camera_info',self.info_callback, 10)

        self.info=None
        self.cvb=CvBridge()

        self.sub_info
        self.sub_image

    def info_callback(self,msg):
        self.info=msg

    def callback(self, msg):
        if self.info is None:
            return

        image=self.cvb.compressed_imgmsg_to_cv2(msg)
        rows,cols=image.shape[:2]
        size=(800,int(800*rows/cols))

        K=np.array(self.info.k).reshape(3,3)
        D=np.array(self.info.d)

        undist_image = cv2.undistort(image, K, D, None, K)
        undist_image = cv2.resize(undist_image, size, fx=0, fy=0, interpolation=cv2.INTER_AREA)

        cv2.imshow('result',undist_image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)

    sub = Subscriber()

    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()