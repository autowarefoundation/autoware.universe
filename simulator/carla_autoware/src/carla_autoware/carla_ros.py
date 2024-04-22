# Copyright 2024 Tier IV, Inc.
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
# limitations under the License.sr/bin/env python

import json
import logging
import math
import threading

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import ControlModeReport
from autoware_auto_vehicle_msgs.msg import GearReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_perception_msgs.msg import TrafficSignalArray
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from transforms3d.euler import euler2quat

from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_data_provider import datetime
from .modules.carla_utils import carla_location_to_ros_point
from .modules.carla_utils import carla_rotation_to_ros_quaternion
from .modules.carla_utils import create_cloud
from .modules.carla_wrapper import SensorInterface


class carla_interface(object):
    def __init__(self):
        self.sensor_interface = SensorInterface()
        self.setup()

    def setup(self):
        self.timestamp = None
        self.channels = 0
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()
        self.first_ = True
        self.sensor_frequencies = {"lidar": 11, "camera": 11, "imu": 50, "status": 50, "pose": 2}
        self.publish_prev_times = {
            sensor: datetime.datetime.now() for sensor in self.sensor_frequencies
        }

        # initialize ros2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("carla_interface")
        self.logger = logging.getLogger("log")
        self.parameters = {
            "host": rclpy.Parameter.Type.STRING,
            "port": rclpy.Parameter.Type.INTEGER,
            "sync_mode": rclpy.Parameter.Type.BOOL,
            "timeout": rclpy.Parameter.Type.INTEGER,
            "fixed_delta_seconds": rclpy.Parameter.Type.DOUBLE,
            "map_name": rclpy.Parameter.Type.STRING,
            "ego_vehicle_role_name": rclpy.Parameter.Type.STRING,
            "spawn_point": rclpy.Parameter.Type.STRING,
            "vehicle_type": rclpy.Parameter.Type.STRING,
            "objects_definition_file": rclpy.Parameter.Type.STRING,
        }
        self.param_values = {}
        for param_name, param_type in self.parameters.items():
            self.ros2_node.declare_parameter(param_name, param_type)
            self.param_values[param_name] = self.ros2_node.get_parameter(param_name).value

        # Publish clock
        self.clock_publisher = self.ros2_node.create_publisher(Clock, "/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)

        # Sensor Config (Edit your sensor here)
        self.sensors = json.load(open(self.param_values["objects_definition_file"]))

        # Subscribing Autoware Control messages and converting to CARLA control
        self.sub_control = self.ros2_node.create_subscription(
            AckermannControlCommand, "/control/command/control_cmd", self.control_callback, 1
        )
        self.current_control = carla.VehicleControl()

        # Direct data publishing from CARLA for Autoware
        self.pub_pose_with_cov = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )
        self.pub_traffic_signal_info = self.ros2_node.create_publisher(
            TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
        )
        self.pub_vel_state = self.ros2_node.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.ros2_node.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.ros2_node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.ros2_node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )

        # Create Publisher for each Physical Sensors
        for sensor in self.sensors["sensors"]:
            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            if sensor["type"] == "sensor.camera.rgb":
                self.pub_camera = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/traffic_light/image_raw", 1
                )
                self.pub_camera_info = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/traffic_light/camera_info", 1
                )
            elif sensor["type"] == "sensor.lidar.ray_cast":
                self.pub_lidar = self.ros2_node.create_publisher(
                    PointCloud2, "/sensing/lidar/top/outlier_filtered/pointcloud", 10
                )
            elif sensor["type"] == "sensor.other.imu":
                self.pub_imu = self.ros2_node.create_publisher(
                    Imu, "/sensing/imu/tamagawa/imu_raw", 1
                )
            else:
                self.logger.info("No Publisher for this Sensor")
                pass

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()

    def __call__(self):
        input_data = self.sensor_interface.get_data()
        timestamp = GameTime.get_time()
        control = self.run_step(input_data, timestamp)
        control.manual_gear_shift = False
        return control

    def get_param(self):
        return self.param_values

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = carla.VehicleControl()
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
        self.current_control = out_cmd

    def checkFrequency(self, sensor):
        time_delta = (
            datetime.datetime.now() - self.publish_prev_times[sensor]
        ).microseconds / 1000000.0
        if 1.0 / time_delta >= self.sensor_frequencies[sensor]:
            return True
        return False

    def get_msg_header(self, frame_id):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def lidar(self, carla_lidar_measurement):
        """Transform the a received lidar measurement into a ROS point cloud message."""
        if self.checkFrequency("lidar"):
            return
        self.publish_prev_times["lidar"] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="velodyne_top")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        ]

        lidar_data = numpy.fromstring(bytes(carla_lidar_measurement.raw_data), dtype=numpy.float32)
        lidar_data = numpy.reshape(lidar_data, (int(lidar_data.shape[0] / 4), 4))

        ring = numpy.empty((0, 1), object)
        self.channels = self.sensors["sensors"]

        for i in range(self.channels[1]["channels"]):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            ring = numpy.vstack((ring, numpy.full((current_ring_points_count, 1), i)))

        lidar_data = numpy.hstack((lidar_data, ring))

        lidar_data[:, 1] *= -1
        point_cloud_msg = create_cloud(header, fields, lidar_data)
        self.pub_lidar.publish(point_cloud_msg)

    def pose(self):
        """Transform odometry data to Pose and publish Pose with Covariance message."""
        if self.checkFrequency("pose"):
            return
        self.publish_prev_times["pose"] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="map")
        ego_ = CarlaDataProvider.get_actor_by_name(self.param_values["ego_vehicle_role_name"])

        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()
        pose_carla.position = carla_location_to_ros_point(
            CarlaDataProvider.get_transform(ego_).location
        )
        pose_carla.orientation = carla_rotation_to_ros_quaternion(
            CarlaDataProvider.get_transform(ego_).rotation
        )
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
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

    def _build_camera_info(self, camera_actor):
        """Build camera info."""
        camera_info = CameraInfo()
        camera_info.width = camera_actor.width
        camera_info.height = camera_actor.height
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(camera_actor.fov * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info = camera_info

    def camera(self, carla_camera_data):
        """Transform the received carla camera data into a ROS image and info message and publish."""
        while self.first_:
            self._camera_info_ = self._build_camera_info(carla_camera_data)
            self.first_ = False

        if self.checkFrequency("camera"):
            return
        self.publish_prev_times["camera"] = datetime.datetime.now()

        image_data_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(frame_id="traffic_light_left_camera/camera_link")
        cam_info = self._camera_info
        cam_info.header = img_msg.header
        self.pub_camera_info.publish(cam_info)
        self.pub_camera.publish(img_msg)

    def imu(self, carla_imu_measurement):
        """Transform a received imu measurement into a ROS Imu message and publish Imu message."""
        if self.checkFrequency("imu"):
            return
        self.publish_prev_times["imu"] = datetime.datetime.now()

        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(frame_id="tamagawa/imu_link")
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll = math.radians(carla_imu_measurement.transform.rotation.roll)
        pitch = -math.radians(carla_imu_measurement.transform.rotation.pitch)
        yaw = -math.radians(carla_imu_measurement.transform.rotation.yaw)

        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        self.pub_imu.publish(imu_msg)

    def ego_status(self):
        """Publish ego vehicle status."""
        if self.checkFrequency("status"):
            return
        self.publish_prev_times["status"] = datetime.datetime.now()

        in_status = CarlaDataProvider.get_actor_by_name(
            self.param_values["ego_vehicle_role_name"]
        ).get_control()
        velocity = CarlaDataProvider.get_actor_by_name(
            self.param_values["ego_vehicle_role_name"]
        ).get_velocity()
        vel_np = numpy.array([velocity.x, velocity.y, velocity.z])
        orientation = numpy.array([1, 1, 0])
        in_vel_state = numpy.abs(numpy.dot(vel_np, orientation))
        self.current_vel = in_vel_state

        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_traffic = TrafficSignalArray()

        out_vel_state.header = self.get_msg_header(frame_id="base_link")
        out_vel_state.longitudinal_velocity = in_vel_state
        out_vel_state.lateral_velocity = 0.0
        out_vel_state.heading_rate = 0.0

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -in_status.steer

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)
        self.pub_traffic_signal_info.publish(out_traffic)

    def run_step(self, input_data, timestamp):
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == "sensor.camera.rgb":
                self.camera(data[1])
            elif sensor_type == "sensor.other.gnss":
                self.pose()
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(data[1])
            elif sensor_type == "sensor.other.imu":
                self.imu(data[1])
            else:
                self.logger.info("No Publisher for [{key}] Sensor")

        # Publish ego vehicle status
        self.ego_status()
        return self.current_control
