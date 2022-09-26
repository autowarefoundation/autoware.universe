# Copyright 2018 Autoware Foundation. All rights reserved.
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

from glob import glob

from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import Shape
from autoware_auto_perception_msgs.msg import TrackedObject
from autoware_auto_perception_msgs.msg import TrackedObjects
from geometry_msgs.msg import TransformStamped
from perception_benchmark_tool.benchmark_tools.datasets.waymo_dataset import WaymoDataset
from perception_benchmark_tool.benchmark_tools.math_utils import rotation_matrix_to_euler_angles
from perception_benchmark_tool.benchmark_tools.ros_utils import create_camera_info
from perception_benchmark_tool.benchmark_tools.ros_utils import create_image_msgs
from perception_benchmark_tool.benchmark_tools.ros_utils import create_point_cloud_mgs
from perception_benchmark_tool.benchmark_tools.ros_utils import make_transform_stamped
import rclpy
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
from unique_identifier_msgs.msg import UUID
from waymo_open_dataset import label_pb2
from waymo_open_dataset.protos import metrics_pb2


def get_tfrecord_paths(path):
    tf_record_list = glob(path + "/*.tfrecord")
    if len(tf_record_list) > 0:
        return tf_record_list
    else:
        return None


class PlayerNode(Node):
    def __init__(self):
        super().__init__("waymo_player_node")

        self.declare_parameter("dataset_path", "")
        dataset_path = self.get_parameter("dataset_path").get_parameter_value().string_value
        self.tf_list = get_tfrecord_paths(dataset_path)

        self.declare_parameter("use_camera", False)
        self.use_camera = self.get_parameter("use_camera").get_parameter_value().bool_value

        self.tf_segment_idx = 0

        self.srv_read_scene_data = self.create_service(
            Trigger, "read_current_segment", self.read_dataset_segment
        )
        self.srv_read_scene_data = self.create_service(
            Trigger, "send_frame", self.frame_processed_callback
        )
        self.pub_segment_finished = self.create_publisher(Bool, "segment_finished", 1)

        self.dataset = None
        self.current_scene_processed = False

        self.pub_lidar_front = self.create_publisher(PointCloud2, "/point_cloud/front_lidar", 10)
        self.pub_lidar_rear = self.create_publisher(PointCloud2, "/point_cloud/rear_lidar", 10)
        self.pub_lidar_side_left = self.create_publisher(
            PointCloud2, "/point_cloud/side_left_lidar", 10
        )
        self.pub_lidar_side_right = self.create_publisher(
            PointCloud2, "/point_cloud/side_right_lidar", 10
        )
        self.pub_lidar_top = self.create_publisher(PointCloud2, "/point_cloud/top_lidar", 10)

        self.pub_gt_objects = self.create_publisher(TrackedObjects, "/gt_objects", 10)

        if self.use_camera:
            self.pub_camera_front = self.create_publisher(Image, "/front_camera", 10)
            self.pub_camera_front_left = self.create_publisher(Image, "/front_left_camera", 10)
            self.pub_camera_front_right = self.create_publisher(Image, "/front_right_camera", 10)
            self.pub_camera_side_left = self.create_publisher(Image, "/side_left_camera", 10)
            self.pub_camera_side_right = self.create_publisher(Image, "/side_right_camera", 10)

            self.pub_cam_info_front = self.create_publisher(CameraInfo, "/front_cam_info", 10)
            self.pub_cam_info_front_left = self.create_publisher(
                CameraInfo, "/front_left_cam_info", 10
            )
            self.pub_cam_info_front_right = self.create_publisher(
                CameraInfo, "/front_right_cam_info", 10
            )
            self.pub_cam_info_side_left = self.create_publisher(
                CameraInfo, "/side_left_cam_info", 10
            )
            self.pub_cam_info_side_right = self.create_publisher(
                CameraInfo, "/side_right_cam_info", 10
            )

        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.pose_broadcaster = TransformBroadcaster(self)
        self.static_tf_publisher = StaticTransformBroadcaster(self)

        self.waymo_evaluation_frame = "base_link"

        self.prediction_proto_objects = metrics_pb2.Objects()
        self.gt_proto_objects = metrics_pb2.Objects()

    def read_dataset_segment(self, request, response):

        if self.tf_segment_idx >= len(self.tf_list):
            self.get_logger().info("All Waymo segments in the given path have been processed.")
            exit()

        self.get_logger().info("Waymo segment decoding from dataset...")
        self.dataset = WaymoDataset(self.tf_list[self.tf_segment_idx])
        self.tf_segment_idx += 1
        response.success = True
        response.message = "Segment readed."
        return response

    def frame_processed_callback(self, request, response):

        if not self.is_dataset_finished():
            self.publish_scene()
            response.success = True
            response.message = "Frame published."
            return response

        else:
            self.get_logger().info("Waymo segment finished.")
            msg = Bool()
            msg.data = True
            self.pub_segment_finished.publish(msg)

            response.success = False
            response.message = "Dataset finished."
            return response

    # Below part copied
    def publish_scene(self):
        self.set_scene_processed(True)

        current_scene = self.dataset.get_scene_from_dataset()
        scene_time_as_ros_time = Time(
            nanoseconds=int(current_scene["TIMESTAMP_MICRO"]) * 1000, clock_type=ClockType.ROS_TIME
        )

        self.publish_static_tf(scene_time_as_ros_time)
        self.publish_pose(current_scene["VEHICLE_POSE"], scene_time_as_ros_time)
        self.publish_lidar_data(current_scene, scene_time_as_ros_time)
        self.publish_gt_objects(current_scene, scene_time_as_ros_time)

        if self.use_camera:
            self.publish_camera_images(current_scene, scene_time_as_ros_time)
            self.publish_camera_info(current_scene, scene_time_as_ros_time)

    def is_dataset_finished(self):
        return self.dataset.is_finished()

    def publish_pose(self, vehicle_pose, ros_time):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = ros_time.to_msg()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"

        rot_mat = vehicle_pose[0:3, 0:3]
        [rx, ry, rz] = rotation_matrix_to_euler_angles(rot_mat)

        transform_stamped.transform.translation.x = float(vehicle_pose[0, 3])
        transform_stamped.transform.translation.y = float(vehicle_pose[1, 3])
        transform_stamped.transform.translation.z = float(vehicle_pose[2, 3])

        quat = tf_transformations.quaternion_from_euler(float(rx), float(ry), float(rz))

        transform_stamped.transform.rotation.x = quat[0]
        transform_stamped.transform.rotation.y = quat[1]
        transform_stamped.transform.rotation.z = quat[2]
        transform_stamped.transform.rotation.w = quat[3]

        self.pose_broadcaster.sendTransform(transform_stamped)

    def publish_static_tf(self, ros_time):
        lidar_transforms = self.dataset.get_lidars_static_tf()

        # Front lidar
        static_ts_front_lidar = make_transform_stamped(
            "base_link", "front_laser", lidar_transforms["FRONT_LASER_EXTRINSIC"], ros_time
        )
        # Rear lidar
        static_ts_rear_lidar = make_transform_stamped(
            "base_link", "rear_laser", lidar_transforms["REAR_LASER_EXTRINSIC"], ros_time
        )
        # Side left lidar
        static_ts_side_left_lidar = make_transform_stamped(
            "base_link", "side_left_laser", lidar_transforms["SIDE_LEFT_LASER_EXTRINSIC"], ros_time
        )
        # Side right lidar
        static_ts_side_right_lidar = make_transform_stamped(
            "base_link",
            "side_right_laser",
            lidar_transforms["SIDE_RIGHT_LASER_EXTRINSIC"],
            ros_time,
        )
        # Top lidar
        static_ts_top_lidar = make_transform_stamped(
            "base_link", "top_laser", lidar_transforms["TOP_LASER_EXTRINSIC"], ros_time
        )

        camera_transforms = self.dataset.get_cameras_static_tf()
        # Front camera
        static_ts_front_camera = make_transform_stamped(
            "base_link", "front_camera", camera_transforms["FRONT_CAM_EXTRINSIC"], ros_time
        )
        # Front left camera
        static_ts_front_left_camera = make_transform_stamped(
            "base_link",
            "front_left_camera",
            camera_transforms["FRONT_LEFT_CAM_EXTRINSIC"],
            ros_time,
        )
        # Front right camera
        static_ts_front_right_camera = make_transform_stamped(
            "base_link",
            "front_right_camera",
            camera_transforms["FRONT_RIGHT_CAM_EXTRINSIC"],
            ros_time,
        )
        # Side left camera
        static_ts_side_left_camera = make_transform_stamped(
            "base_link", "side_left_camera", camera_transforms["SIDE_LEFT_CAM_EXTRINSIC"], ros_time
        )
        # Side right camera
        static_ts_side_right_camera = make_transform_stamped(
            "base_link",
            "side_right_camera",
            camera_transforms["SIDE_RIGHT_CAM_EXTRINSIC"],
            ros_time,
        )

        self.static_tf_publisher.sendTransform(
            [
                static_ts_front_lidar,
                static_ts_rear_lidar,
                static_ts_side_left_lidar,
                static_ts_side_right_lidar,
                static_ts_top_lidar,
                static_ts_front_camera,
                static_ts_front_left_camera,
                static_ts_front_right_camera,
                static_ts_side_left_camera,
                static_ts_side_right_camera,
            ]
        )

    def publish_camera_images(self, current_scene, ros_time_now):
        self.pub_camera_front.publish(
            create_image_msgs("front_camera", current_scene["FRONT_IMAGE"], ros_time_now)
        )
        self.pub_camera_front_left.publish(
            create_image_msgs("front_left_camera", current_scene["FRONT_LEFT_IMAGE"], ros_time_now)
        )
        self.pub_camera_front_right.publish(
            create_image_msgs(
                "front_right_camera", current_scene["FRONT_RIGHT_IMAGE"], ros_time_now
            )
        )
        self.pub_camera_side_left.publish(
            create_image_msgs("side_left_camera", current_scene["SIDE_LEFT_IMAGE"], ros_time_now)
        )
        self.pub_camera_side_right.publish(
            create_image_msgs("side_right_camera", current_scene["SIDE_RIGHT_IMAGE"], ros_time_now)
        )

    def publish_camera_info(self, current_scene, ros_time_now):
        self.pub_cam_info_front.publish(
            create_camera_info("base_link", current_scene["FRONT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_front_left.publish(
            create_camera_info("base_link", current_scene["FRONT_LEFT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_front_right.publish(
            create_camera_info("base_link", current_scene["FRONT_RIGHT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_side_left.publish(
            create_camera_info("base_link", current_scene["SIDE_LEFT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_side_right.publish(
            create_camera_info("base_link", current_scene["SIDE_RIGHT_CAM_INFO"], ros_time_now)
        )

    def publish_lidar_data(self, current_scene, ros_time_now):
        self.pub_lidar_top.publish(
            create_point_cloud_mgs("base_link", current_scene["TOP_LASER"], ros_time_now)
        )
        self.pub_lidar_front.publish(
            create_point_cloud_mgs("base_link", current_scene["FRONT_LASER"], ros_time_now)
        )
        self.pub_lidar_side_left.publish(
            create_point_cloud_mgs("base_link", current_scene["SIDE_LEFT_LASER"], ros_time_now)
        )
        self.pub_lidar_side_right.publish(
            create_point_cloud_mgs("base_link", current_scene["SIDE_RIGHT_LASER"], ros_time_now)
        )
        self.pub_lidar_rear.publish(
            create_point_cloud_mgs("base_link", current_scene["REAR_LASER"], ros_time_now)
        )

    def publish_gt_objects(self, current_scene, ros_time):

        ground_truth_objects = TrackedObjects()
        ground_truth_objects.header.frame_id = "base_link"
        ground_truth_objects.header.stamp = ros_time.to_msg()

        for gt_object in current_scene["GT_OBJECTS"]:

            if gt_object.num_lidar_points_in_box <= 0:
                continue

            gt_detected_object = TrackedObject()
            object_classification = ObjectClassification()
            gt_detected_object.existence_probability = 1.0

            if gt_object.type == label_pb2.Label.TYPE_VEHICLE:
                object_classification.label = ObjectClassification.CAR
                gt_detected_object.shape.type = Shape.BOUNDING_BOX
            elif gt_object.type == label_pb2.Label.TYPE_PEDESTRIAN:
                object_classification.label = ObjectClassification.PEDESTRIAN
                gt_detected_object.shape.type = Shape.CYLINDER
            elif gt_object.type == label_pb2.Label.TYPE_CYCLIST:
                object_classification.label = ObjectClassification.BICYCLE
                gt_detected_object.shape.type = Shape.BOUNDING_BOX
            else:
                continue

            gt_detected_object.classification.append(object_classification)

            # Pedestrian bounding boxes x and y fixed in Autoware
            if gt_object.type == label_pb2.Label.TYPE_PEDESTRIAN:
                gt_detected_object.shape.dimensions.x = 1.0
                gt_detected_object.shape.dimensions.y = 1.0
                gt_detected_object.shape.dimensions.z = gt_object.box.height
            else:
                gt_detected_object.shape.dimensions.x = gt_object.box.length
                gt_detected_object.shape.dimensions.y = gt_object.box.width
                gt_detected_object.shape.dimensions.z = gt_object.box.height

            gt_detected_object.kinematics.pose_with_covariance.pose.position.x = (
                gt_object.box.center_x
            )
            gt_detected_object.kinematics.pose_with_covariance.pose.position.y = (
                gt_object.box.center_y
            )
            gt_detected_object.kinematics.pose_with_covariance.pose.position.z = (
                gt_object.box.center_z
            )

            q = tf_transformations.quaternion_from_euler(0, 0, gt_object.box.heading)

            gt_detected_object.kinematics.pose_with_covariance.pose.orientation.x = q[0]
            gt_detected_object.kinematics.pose_with_covariance.pose.orientation.y = q[1]
            gt_detected_object.kinematics.pose_with_covariance.pose.orientation.z = q[2]
            gt_detected_object.kinematics.pose_with_covariance.pose.orientation.w = q[3]

            str_1_encoded = gt_object.id.encode(encoding="UTF-8")
            uuid_msg = UUID()

            for i in range(16):
                uuid_msg.uuid[i] = str_1_encoded[i]

            gt_detected_object.object_id = uuid_msg
            ground_truth_objects.objects.append(gt_detected_object)

        self.pub_gt_objects.publish(ground_truth_objects)

    def scene_processed(self):
        return self.current_scene_processed

    def set_scene_processed(self, value):
        self.current_scene_processed = value


def main(args=None):
    rclpy.init(args=args)
    perception_benchmark = PlayerNode()
    rclpy.spin(perception_benchmark)
    perception_benchmark.destroy_node()
    rclpy.shutdown()
