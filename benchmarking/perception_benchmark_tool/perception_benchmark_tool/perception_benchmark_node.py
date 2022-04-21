import math
import threading

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
from perception_benchmark_tool.benchmark_tools.waymo_ros_formatter.waymo_handler import WaymoHandler
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations


class WaymoPublisherNode(Node):
    def __init__(self):
        super().__init__("perception_benchmark_node")
        self.declare_parameter("file_path")
        self.param_path = self.get_parameter("file_path").get_parameter_value().string_value

        self.pub_camera_front = self.create_publisher(Image, "front_image", qos_profile_sensor_data)
        self.pub_camera_front_left = self.create_publisher(
            Image, "front_left_image", qos_profile_sensor_data
        )
        self.pub_camera_front_right = self.create_publisher(
            Image, "front_right_image", qos_profile_sensor_data
        )
        self.pub_camera_side_left = self.create_publisher(
            Image, "side_left_image", qos_profile_sensor_data
        )
        self.pub_camera_side_right = self.create_publisher(
            Image, "side_right_image", qos_profile_sensor_data
        )

        self.pub_cam_info_front = self.create_publisher(CameraInfo, "front_cam_info", 1)
        self.pub_cam_info_front_left = self.create_publisher(CameraInfo, "front_left_cam_info", 1)
        self.pub_cam_info_front_right = self.create_publisher(CameraInfo, "front_right_cam_info", 1)
        self.pub_cam_info_side_left = self.create_publisher(CameraInfo, "side_left_cam_info", 1)
        self.pub_cam_info_side_right = self.create_publisher(CameraInfo, "side_right_cam_info", 1)

        self.pub_lidar_front = self.create_publisher(
            PointCloud2, "/point_cloud/front_lidar", qos_profile_sensor_data
        )
        self.pub_lidar_rear = self.create_publisher(
            PointCloud2, "/point_cloud/rear_lidar", qos_profile_sensor_data
        )
        self.pub_lidar_side_left = self.create_publisher(
            PointCloud2, "/point_cloud/side_left_lidar", qos_profile_sensor_data
        )
        self.pub_lidar_side_right = self.create_publisher(
            PointCloud2, "/point_cloud/side_right_lidar", qos_profile_sensor_data
        )
        self.pub_lidar_top = self.create_publisher(
            PointCloud2, "/point_cloud/top_lidar", qos_profile_sensor_data
        )

        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.bridge = CvBridge()
        self.dataset = WaymoHandler(self.param_path)

        self.pose_broadcaster = TransformBroadcaster(self)
        self.static_tf_publisher = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_waymo_data(self):
        current_scene = self.dataset.get_scene_from_dataset()
        ros_time_now = self.get_clock().now()
        self.publish_pose(current_scene["VEHICLE_POSE"], ros_time_now)
        self.publish_camera_images(current_scene, ros_time_now)
        self.publish_camera_info(current_scene, ros_time_now)
        self.publish_lidar_data(current_scene, ros_time_now)

    def create_header(self, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def is_dataset_finished(self):
        return self.dataset.is_finished()

    def publish_pose(self, vehicle_pose, ros_time):

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = ros_time.to_msg()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"

        rot_mat = vehicle_pose[0:3, 0:3]
        [rx, ry, rz] = self.rotation_matrix_to_euler_angles(rot_mat)

        transform_stamped.transform.translation.x = float(vehicle_pose[0, 3])
        transform_stamped.transform.translation.y = float(vehicle_pose[1, 3])
        transform_stamped.transform.translation.z = float(vehicle_pose[2, 3])

        quat = tf_transformations.quaternion_from_euler(float(rx), float(ry), float(rz))

        transform_stamped.transform.rotation.x = quat[0]
        transform_stamped.transform.rotation.y = quat[1]
        transform_stamped.transform.rotation.z = quat[2]
        transform_stamped.transform.rotation.w = quat[3]

        self.pose_broadcaster.sendTransform(transform_stamped)

    def publish_static_tf(self):

        lidar_transforms = self.dataset.get_lidars_static_tf()

        # Front lidar
        static_ts_front_lidar = self.make_transform_stamped(
            "base_link", "front_laser", lidar_transforms["FRONT_LASER_EXTRINSIC"]
        )
        # Rear lidar
        static_ts_rear_lidar = self.make_transform_stamped(
            "base_link", "rear_laser", lidar_transforms["REAR_LASER_EXTRINSIC"]
        )
        # Side left lidar
        static_ts_side_left_lidar = self.make_transform_stamped(
            "base_link", "side_left_laser", lidar_transforms["SIDE_LEFT_LASER_EXTRINSIC"]
        )
        # Side right lidar
        static_ts_side_right_lidar = self.make_transform_stamped(
            "base_link", "side_right_laser", lidar_transforms["SIDE_RIGHT_LASER_EXTRINSIC"]
        )
        # Top lidar
        static_ts_top_lidar = self.make_transform_stamped(
            "base_link", "top_laser", lidar_transforms["TOP_LASER_EXTRINSIC"]
        )

        camera_transforms = self.dataset.get_cameras_static_tf()
        # Front camera
        static_ts_front_camera = self.make_transform_stamped(
            "base_link", "front_camera", camera_transforms["FRONT_CAM_EXTRINSIC"]
        )
        # Front left camera
        static_ts_front_left_camera = self.make_transform_stamped(
            "base_link", "front_left_camera", camera_transforms["FRONT_LEFT_CAM_EXTRINSIC"]
        )
        # Front right camera
        static_ts_front_right_camera = self.make_transform_stamped(
            "base_link", "front_right_camera", camera_transforms["FRONT_RIGHT_CAM_EXTRINSIC"]
        )
        # Side left camera
        static_ts_side_left_camera = self.make_transform_stamped(
            "base_link", "side_left_camera", camera_transforms["SIDE_LEFT_CAM_EXTRINSIC"]
        )
        # Side right camera
        static_ts_side_right_camera = self.make_transform_stamped(
            "base_link", "side_right_camera", camera_transforms["SIDE_RIGHT_CAM_EXTRINSIC"]
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

    def make_transform_stamped(self, header_frame_id, child_frame_id, lidar_transform):
        rot_mat = lidar_transform[0:3, 0:3]
        [rx, ry, rz] = self.rotation_matrix_to_euler_angles(rot_mat)
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = header_frame_id
        static_transform_stamped.child_frame_id = child_frame_id
        static_transform_stamped.transform.translation.x = float(lidar_transform[0, 3])
        static_transform_stamped.transform.translation.y = float(lidar_transform[1, 3])
        static_transform_stamped.transform.translation.z = float(lidar_transform[2, 3])
        quat = tf_transformations.quaternion_from_euler(rx, ry, rz)
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        return static_transform_stamped

    def is_rotation_matrix(self, R):

        r_transpose = np.transpose(R)
        should_be_identity = np.dot(r_transpose, R)
        identity = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(identity - should_be_identity)
        return n < 1e-6

    def rotation_matrix_to_euler_angles(self, R):

        assert self.is_rotation_matrix(R)

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def create_point_cloud_mgs(self, frame_id, lidar_frame, ros_time):

        msg = PointCloud2()
        msg.header.stamp = ros_time.to_msg()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(lidar_frame)
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = False
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        # We pass `data` directly to we avoid using `data` setter.
        # Otherwise ROS2 converts data to `array.array` which slows down as it copies memory internally.
        # Both, `bytearray` and `array.array`, implement Python buffer protocol, so we should not see unpredictable
        # behavior.
        # deepcode ignore W0212: Avoid conversion from `bytearray` to `array.array`.
        msg._data = lidar_frame
        return msg

    def create_image_msgs(self, frame_id, camera_frame, ros_time):
        image_mgs = self.bridge.cv2_to_imgmsg(camera_frame, encoding="bgr8")
        image_mgs.header.stamp = ros_time.to_msg()
        image_mgs.header.frame_id = frame_id
        return image_mgs

    def create_camera_info(self, frame_id, camera_calibration, ros_time):
        camera_info = CameraInfo()
        # cam = P2 * RO_rect * Tr_velo_to_cam * velo
        camera_info.header.frame_id = frame_id
        camera_info.header.stamp = ros_time.to_msg()
        camera_info.width = camera_calibration["width"]
        camera_info.height = camera_calibration["height"]
        camera_info.p = (
            np.array(camera_calibration["vehicle_to_image"]).astype(float).flatten().tolist()
        )
        return camera_info

    def cart_to_homo(self, mat):
        ret = np.eye(4)
        if mat.shape == (3, 3):
            ret[:3, :3] = mat
        elif mat.shape == (3, 4):
            ret[:3, :] = mat
        else:
            raise ValueError(mat.shape)
        return ret

    def publish_camera_images(self, current_scene, ros_time_now):
        self.pub_camera_front.publish(
            self.create_image_msgs("front_camera", current_scene["FRONT_IMAGE"], ros_time_now)
        )
        self.pub_camera_front_left.publish(
            self.create_image_msgs(
                "front_left_camera", current_scene["FRONT_LEFT_IMAGE"], ros_time_now
            )
        )
        self.pub_camera_front_right.publish(
            self.create_image_msgs(
                "front_right_camera", current_scene["FRONT_RIGHT_IMAGE"], ros_time_now
            )
        )
        self.pub_camera_side_left.publish(
            self.create_image_msgs(
                "side_left_camera", current_scene["SIDE_LEFT_IMAGE"], ros_time_now
            )
        )
        self.pub_camera_side_right.publish(
            self.create_image_msgs(
                "side_right_camera", current_scene["SIDE_RIGHT_IMAGE"], ros_time_now
            )
        )

    def publish_camera_info(self, current_scene, ros_time_now):
        self.pub_cam_info_front.publish(
            self.create_camera_info("base_link", current_scene["FRONT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_front_left.publish(
            self.create_camera_info("base_link", current_scene["FRONT_LEFT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_front_right.publish(
            self.create_camera_info(
                "base_link", current_scene["FRONT_RIGHT_CAM_INFO"], ros_time_now
            )
        )
        self.pub_cam_info_side_left.publish(
            self.create_camera_info("base_link", current_scene["SIDE_LEFT_CAM_INFO"], ros_time_now)
        )
        self.pub_cam_info_side_right.publish(
            self.create_camera_info("base_link", current_scene["SIDE_RIGHT_CAM_INFO"], ros_time_now)
        )

    def publish_lidar_data(self, current_scene, ros_time_now):
        self.pub_lidar_top.publish(
            self.create_point_cloud_mgs("base_link", current_scene["TOP_LASER"], ros_time_now)
        )
        self.pub_lidar_front.publish(
            self.create_point_cloud_mgs("base_link", current_scene["FRONT_LASER"], ros_time_now)
        )
        self.pub_lidar_side_left.publish(
            self.create_point_cloud_mgs("base_link", current_scene["SIDE_LEFT_LASER"], ros_time_now)
        )
        self.pub_lidar_side_right.publish(
            self.create_point_cloud_mgs(
                "base_link", current_scene["SIDE_RIGHT_LASER"], ros_time_now
            )
        )
        self.pub_lidar_rear.publish(
            self.create_point_cloud_mgs("base_link", current_scene["REAR_LASER"], ros_time_now)
        )


def main(args=None):
    rclpy.init(args=args)

    node = WaymoPublisherNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(10, node.get_clock())

    while rclpy.ok() and not node.is_dataset_finished():
        node.publish_waymo_data()
        rate.sleep()

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
