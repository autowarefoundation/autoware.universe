from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport
from autoware_auto_vehicle_msgs.msg import GearReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
import carla
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2


class CarlaVehicleInterface(Node):
    def __init__(self):
        super().__init__("carla_vehicle_interface_node")

        client = carla.Client("localhost", 2000)
        client.set_timeout(30)

        self._world = client.get_world()
        self.current_vel = 0.0
        self.target_vel = 0.0
        self.vel_diff = 0.0
        self.current_control = carla.VehicleControl()

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
        self.vehicle_imu_publisher = self.create_publisher(Imu, "/sensing/imu/tamagawa/imu_raw", 1)
        self.sensing_cloud_publisher = self.create_publisher(PointCloud2, "/carla_pointcloud", 1)

        # Subscribe Topics used in Control
        self.sub_status = self.create_subscription(
            CarlaEgoVehicleStatus, "/carla/ego_vehicle/vehicle_status", self.ego_status_callback, 1
        )
        self.sub_control = self.create_subscription(
            AckermannControlCommand,
            "/control/command/control_cmd",
            self.control_callback,
            qos_profile=QoSProfile(depth=1),
        )
        self.sub_imu = self.create_subscription(Imu, "/carla/ego_vehicle/imu", self.publish_imu, 1)
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            "/carla/ego_vehicle/lidar",
            self.publish_lidar,
            qos_profile=QoSProfile(depth=1),
        )

    def ego_status_callback(self, in_status):
        """Convert and publish CARLA Ego Vehicle Status to AUTOWARE."""
        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()

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

    def publish_lidar(self, lidar_msg):
        """Publish LIDAR to Interface."""
        lidar_msg.header.frame_id = "velodyne_top"
        self.sensing_cloud_publisher.publish(lidar_msg)

    def publish_imu(self, imu_msg):
        """Publish IMU to Autoware."""
        imu_msg.header.frame_id = "tamagawa/imu_link"
        self.vehicle_imu_publisher.publish(imu_msg)


def main(args=None):
    rclpy.init()
    node = CarlaVehicleInterface()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
