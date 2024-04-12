from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport
from autoware_auto_vehicle_msgs.msg import GearReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_perception_msgs.msg import TrafficSignalArray
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class CarlaVehicleInterface(Node):
    def __init__(self):
        super().__init__("carla_vehicle_interface_node")
        self.current_vel = 0.0
        self.target_vel = 0.0
        self.vel_diff = 0.0

        # Publishes Topics used for AUTOWARE
        self.pub_vel_state = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.create_publisher(GearReport, "/vehicle/status/gear_status", 1)
        self.pub_control = self.create_publisher(
            CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", 1
        )
        self.pub_traffic_signal_info = self.create_publisher(
            TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
        )
        self.pub_pose_with_cov = self.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )

        # Subscribe Topics used in Control
        self.sub_status = self.create_subscription(
            CarlaEgoVehicleStatus, "/carla/ego_vehicle/vehicle_status", self.ego_status_callback, 1
        )
        self.sub_control = self.create_subscription(
            AckermannControlCommand, "/control/command/control_cmd", self.control_callback, 1
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/carla/ego_vehicle/odometry", self.pose_callback, 1
        )

    def pose_callback(self, pose_msg):
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = "map"
        out_pose_with_cov.header.stamp = pose_msg.header.stamp
        out_pose_with_cov.pose.pose = pose_msg.pose.pose
        out_pose_with_cov.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_pose_with_cov.publish(out_pose_with_cov)

    def ego_status_callback(self, in_status):
        """Convert and publish CARLA Ego Vehicle Status to AUTOWARE."""
        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_traffic = TrafficSignalArray()

        out_vel_state.header = in_status.header
        out_vel_state.header.frame_id = "base_link"
        out_vel_state.longitudinal_velocity = in_status.velocity
        out_vel_state.lateral_velocity = 0.0
        out_vel_state.heading_rate = 0.0
        self.current_vel = in_status.velocity

        out_steering_state.stamp = in_status.header.stamp
        out_steering_state.steering_tire_angle = -in_status.control.steer

        out_gear_state.stamp = in_status.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = in_status.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)
        self.pub_traffic_signal_info.publish(out_traffic)

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = CarlaEgoVehicleControl()
        self.target_vel = abs(in_cmd.longitudinal.speed)
        self.vel_diff = self.target_vel - self.current_vel

        if self.vel_diff > 0:
            out_cmd.throttle = 0.75
            out_cmd.brake = 0.0
        elif self.vel_diff <= 0.0:
            out_cmd.throttle = 0.0
            if self.target_vel <= 0.0:
                out_cmd.brake = 0.75
            elif self.vel_diff > -1:
                out_cmd.brake = 0.0
            else:
                out_cmd.brake = 0.01

        out_cmd.steer = -in_cmd.lateral.steering_tire_angle
        self.pub_control.publish(out_cmd)


def main(args=None):
    rclpy.init()
    node = CarlaVehicleInterface()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
