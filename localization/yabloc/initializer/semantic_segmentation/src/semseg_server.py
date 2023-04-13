#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import semseg_core
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge
import time
from semseg_msgs.srv import Semseg


class SemsegServer(Node):
    def __init__(self):
        super().__init__('semseg_node')

        model_path = self.declare_parameter('model_path', '').value

        self.get_logger().info('model path: ' + model_path)
        self.dnn_ = semseg_core.SemSeg(model_path)
        self.bridge_ = CvBridge()

        self.srv = self.create_service(
            Semseg, '/srv/semseg_srv', self.on_service)

    def on_service(self, request, response):
        response.dst_image = self.__inference(request.src_image)
        return response

    def __inference(self, msg: Image):
        stamp = msg.header.stamp
        self.get_logger().info('Subscribed image: ' + str(stamp))

        src_image = self.bridge_.imgmsg_to_cv2(msg)
        start_time = time.time()
        mask = self.dnn_.inference(src_image)
        elapsed_time = time.time() - start_time

        dst_msg = self.bridge_.cv2_to_imgmsg(mask)
        dst_msg.encoding = 'bgr8'
        return dst_msg


def main():
    rclpy.init(args=sys.argv)

    semseg_node = SemsegServer()
    rclpy.spin(semseg_node)
    semseg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
