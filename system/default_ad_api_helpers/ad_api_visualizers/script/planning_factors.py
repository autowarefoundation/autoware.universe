#!/usr/bin/env python3

from autoware_ad_api_msgs.msg import VelocityFactor
from autoware_ad_api_msgs.msg import VelocityFactorArray
import rclpy
import rclpy.duration
import rclpy.node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

velocity_type_text = {
    VelocityFactor.UNKNOWN: "unknown",
    VelocityFactor.SURROUNDING_OBSTACLE: "surrounding obstacle",
    VelocityFactor.ROUTE_OBSTACLE: "route obstacle",
    VelocityFactor.INTERSECTION: "intersection",
    VelocityFactor.CROSSWALK: "crosswalk",
    VelocityFactor.REAR_CHECK: "rear check",
    VelocityFactor.USER_DEFINED_DETECTION_AREA: "detection area",
    VelocityFactor.NO_STOPPING_AREA: "no stopping area",
    VelocityFactor.STOP_SIGN: "stop sign",
    VelocityFactor.TRAFFIC_SIGNAL: "traffic signal",
    VelocityFactor.V2I_GATE_CONTROL_ENTER: "v2i enter",
    VelocityFactor.V2I_GATE_CONTROL_LEAVE: "v2i leave",
    VelocityFactor.MERGE: "merge",
    VelocityFactor.SIDEWALK: "sidewalk",
    VelocityFactor.LANE_CHANGE: "lane change",
    VelocityFactor.AVOIDANCE: "avoidance",
    VelocityFactor.EMERGENCY_STOP_OPERATION: "emergency stop operation",
}

velocity_status_color = {
    VelocityFactor.UNKNOWN: (1.0, 1.0, 1.0),
    VelocityFactor.APPROACHING: (1.0, 0.0, 0.0),
    VelocityFactor.STOPPED: (1.0, 1.0, 0.0),
}


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
            markers.markers.append(self.create_wall_marker(index, msg.header, factor))
            markers.markers.append(self.create_text_marker(index, msg.header, factor))
        self.pub.publish(markers)

    @staticmethod
    def create_wall_marker(index, header, factor):
        marker = Marker()
        marker.ns = "velocity_factors/wall"
        marker.id = index
        marker.lifetime = rclpy.duration.Duration(nanoseconds=0.3 * 1e9).to_msg()
        marker.header = header
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.pose.position.x = factor.pose.position.x
        marker.pose.position.y = factor.pose.position.y
        marker.pose.position.z = factor.pose.position.z + 1.0
        marker.pose.orientation = factor.pose.orientation
        marker.scale.x = 0.1
        marker.scale.y = 5.0
        marker.scale.z = 2.0
        marker.color.a = 0.7
        marker.color.r = velocity_status_color[factor.status][0]
        marker.color.g = velocity_status_color[factor.status][1]
        marker.color.b = velocity_status_color[factor.status][2]
        return marker

    @staticmethod
    def create_text_marker(index, header, factor):
        marker = Marker()
        marker.ns = "velocity_factors/text"
        marker.id = index
        marker.lifetime = rclpy.duration.Duration(nanoseconds=0.3 * 1e9).to_msg()
        marker.header = header
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = velocity_type_text[factor.type]
        marker.pose.position.x = factor.pose.position.x
        marker.pose.position.y = factor.pose.position.y
        marker.pose.position.z = factor.pose.position.z + 2.0
        marker.pose.orientation = factor.pose.orientation
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = PlanningFactorVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
