#!/usr/bin/env python3

from autoware_ad_api_msgs.msg import VelocityFactorArray
import rclpy
import rclpy.duration
import rclpy.node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PlanningFactorVisualizer(rclpy.node.Node):
    def __init__(self):
        super().__init__("planning_factor_visualizer")
        self.sub = self.create_subscription(
            VelocityFactorArray, "/api/planning/velocity_factors", self.on_velocity_factor, 1
        )
        self.pub = self.create_publisher(MarkerArray, "/visualizer/velocity_factors", 1)

    def on_velocity_factor(self, msg):
        markers = MarkerArray()
        for index, factor in enumerate(msg.factors):
            print(index, factor.pose)
            marker = self.create_wall_marker(index, msg.header, factor.pose)
            markers.markers.append(marker)
        self.pub.publish(markers)

    @staticmethod
    def create_wall_marker(index, header, pose):
        marker = Marker()
        marker.ns = "velocity_factors"
        marker.id = index
        marker.lifetime = rclpy.duration.Duration(nanoseconds=0.3 * 1e9).to_msg()
        marker.header = header
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.pose = pose
        marker.pose.position.z += 1.0
        marker.scale.x = 0.1
        marker.scale.y = 4.0
        marker.scale.z = 2.0
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = PlanningFactorVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
