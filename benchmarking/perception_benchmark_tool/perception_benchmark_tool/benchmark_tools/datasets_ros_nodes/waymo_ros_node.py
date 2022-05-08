from autoware_auto_perception_msgs.msg import DetectedObject
from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import Shape
from autoware_auto_perception_msgs.msg import TrackedObjects
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from perception_benchmark_tool.benchmark_tools.datasets.waymo_dataset import WaymoDataset
from perception_benchmark_tool.benchmark_tools.math_utils import euler_from_quaternion
from perception_benchmark_tool.benchmark_tools.math_utils import rotation_matrix_to_euler_angles
from perception_benchmark_tool.benchmark_tools.ros_utils import create_camera_info
from perception_benchmark_tool.benchmark_tools.ros_utils import create_image_msgs
from perception_benchmark_tool.benchmark_tools.ros_utils import create_point_cloud_mgs
from perception_benchmark_tool.benchmark_tools.ros_utils import do_transform_pose_stamped
from perception_benchmark_tool.benchmark_tools.ros_utils import make_transform_stamped
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from waymo_open_dataset import label_pb2
from waymo_open_dataset.protos import metrics_pb2


class WaymoRosNode(Node):
    def __init__(self, waymo_scene_path):
        super().__init__("perception_benchmark_node")

        # self.declare_parameter("file_path")
        # self.param_path = self.get_parameter("file_path").get_parameter_value().string_value
        self.declare_parameter("prediction_path", "")
        self.prediction_path = (
            self.get_parameter("prediction_path").get_parameter_value().string_value
        )

        self.declare_parameter("ground_truth_path", "")
        self.gt_path = self.get_parameter("ground_truth_path").get_parameter_value().string_value

        self.param_path = waymo_scene_path

        self.context_waymo = {}
        self.scene_timestamp = {}
        self.current_scene_processed = False

        self.pub_camera_front = self.create_publisher(Image, "/front_camera", 10)
        self.pub_camera_front_left = self.create_publisher(Image, "/front_left_camera", 10)
        self.pub_camera_front_right = self.create_publisher(Image, "/front_right_camera", 10)
        self.pub_camera_side_left = self.create_publisher(Image, "/side_left_camera", 10)
        self.pub_camera_side_right = self.create_publisher(Image, "/side_right_camera", 10)

        self.pub_cam_info_front = self.create_publisher(CameraInfo, "/front_cam_info", 10)
        self.pub_cam_info_front_left = self.create_publisher(CameraInfo, "/front_left_cam_info", 10)
        self.pub_cam_info_front_right = self.create_publisher(
            CameraInfo, "/front_right_cam_info", 10
        )
        self.pub_cam_info_side_left = self.create_publisher(CameraInfo, "/side_left_cam_info", 10)
        self.pub_cam_info_side_right = self.create_publisher(CameraInfo, "/side_right_cam_info", 10)

        self.pub_lidar_front = self.create_publisher(PointCloud2, "/point_cloud/front_lidar", 10)
        self.pub_lidar_rear = self.create_publisher(PointCloud2, "/point_cloud/rear_lidar", 10)
        self.pub_lidar_side_left = self.create_publisher(
            PointCloud2, "/point_cloud/side_left_lidar", 10
        )
        self.pub_lidar_side_right = self.create_publisher(
            PointCloud2, "/point_cloud/side_right_lidar", 10
        )
        self.pub_lidar_top = self.create_publisher(PointCloud2, "/point_cloud/top_lidar", 10)

        self.pub_gt_objects = self.create_publisher(DetectedObjects, "/gt_objects", 10)

        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.dataset = WaymoDataset(self.param_path)
        self.pose_broadcaster = TransformBroadcaster(self)
        self.static_tf_publisher = StaticTransformBroadcaster(self)
        self.sub_tracking = self.create_subscription(
            TrackedObjects,
            "/perception/object_recognition/tracking/objects",
            self.tracked_objects_callback,
            10,
        )

        self.waymo_evaluation_frame = "base_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prediction_proto_objects = metrics_pb2.Objects()
        self.gt_proto_objects = metrics_pb2.Objects()

    def tracked_objects_callback(self, tracked_objects):

        for tracked_object in tracked_objects.objects:
            if tracked_objects.header.frame_id != self.waymo_evaluation_frame:
                tracked_object = self.transform_tracked_object(
                    tracked_object, tracked_objects.header
                )

            tracked_object_waymo = metrics_pb2.Object()
            tracked_object_waymo.context_name = self.context_waymo
            tracked_object_waymo.frame_timestamp_micros = self.scene_timestamp
            bbox_waymo = label_pb2.Label.Box()
            bbox_waymo.center_x = tracked_object.kinematics.pose_with_covariance.pose.position.x
            bbox_waymo.center_y = tracked_object.kinematics.pose_with_covariance.pose.position.y
            bbox_waymo.center_z = tracked_object.kinematics.pose_with_covariance.pose.position.z
            bbox_waymo.length = tracked_object.shape.dimensions.x
            bbox_waymo.width = tracked_object.shape.dimensions.y
            bbox_waymo.height = tracked_object.shape.dimensions.z

            roll, pitch, yaw = euler_from_quaternion(
                tracked_object.kinematics.pose_with_covariance.pose.orientation.x,
                tracked_object.kinematics.pose_with_covariance.pose.orientation.y,
                tracked_object.kinematics.pose_with_covariance.pose.orientation.z,
                tracked_object.kinematics.pose_with_covariance.pose.orientation.w,
            )

            bbox_waymo.heading = yaw
            tracked_object_waymo.object.box.CopyFrom(bbox_waymo)
            tracked_object_waymo.score = 0.5
            if tracked_object.classification[0].label == ObjectClassification.CAR:
                tracked_object_waymo.object.type = label_pb2.Label.TYPE_VEHICLE
            elif tracked_object.classification[0].label == ObjectClassification.PEDESTRIAN:
                tracked_object_waymo.object.type = label_pb2.Label.TYPE_PEDESTRIAN
            elif tracked_object.classification[0].label == ObjectClassification.BICYCLE:
                tracked_object_waymo.object.type = label_pb2.Label.TYPE_CYCLIST
            else:
                continue

            object_track_id = "".join(
                str(tracked_object.object_id.uuid[e])
                for e in range(0, len(tracked_object.object_id.uuid))
            )
            tracked_object_waymo.object.id = str(object_track_id)

            self.prediction_proto_objects.objects.append(tracked_object_waymo)

        if self.is_dataset_finished():
            with open(self.prediction_path, "ab+") as prediction_file:
                prediction_file.write(self.prediction_proto_objects.SerializeToString())

        self.set_scene_processed(False)

    def transform_tracked_object(self, tracked_object, object_header):

        tracked_objected_map_pose = PoseStamped()
        tracked_objected_map_pose.header.stamp = object_header.stamp
        tracked_objected_map_pose.header.frame_id = object_header.frame_id
        tracked_objected_map_pose.pose.position = (
            tracked_object.kinematics.pose_with_covariance.pose.position
        )
        tracked_objected_map_pose.pose.orientation = (
            tracked_object.kinematics.pose_with_covariance.pose.orientation
        )

        try:
            trans = self.tf_buffer.lookup_transform("base_link", "map", object_header.stamp)
        except TransformException as ex:
            self.get_logger().info("Could not find transform:" + str(ex))
            return

        object_transformed = do_transform_pose_stamped(tracked_objected_map_pose, trans)
        # object_transformed = tf2_geometry_msgs.do_transform_pose_stamped(tracked_objected_map_pose, trans)

        tracked_object_trans = tracked_object
        tracked_object_trans.kinematics.pose_with_covariance.pose.orientation = (
            object_transformed.pose.orientation
        )
        tracked_object_trans.kinematics.pose_with_covariance.pose.position = (
            object_transformed.pose.position
        )

        return tracked_object_trans

    def publish_scene(self):
        self.set_scene_processed(True)

        current_scene = self.dataset.get_scene_from_dataset()
        scene_time_as_ros_time = Time(
            nanoseconds=int(current_scene["TIMESTAMP_MICRO"]) * 1000, clock_type=ClockType.ROS_TIME
        )
        self.scene_timestamp = current_scene["TIMESTAMP_MICRO"]
        self.context_waymo = current_scene["FRAME_CONTEXT_NAME"]

        self.publish_static_tf(scene_time_as_ros_time)
        self.publish_pose(current_scene["VEHICLE_POSE"], scene_time_as_ros_time)
        self.publish_camera_images(current_scene, scene_time_as_ros_time)
        self.publish_camera_info(current_scene, scene_time_as_ros_time)
        self.publish_lidar_data(current_scene, scene_time_as_ros_time)
        self.publish_gt_objects(current_scene, scene_time_as_ros_time)

    def check_subscriber_ready(self):

        lidar_subscribers_ready = (
            self.pub_lidar_front.get_subscription_count()
            and self.pub_lidar_rear.get_subscription_count()
            and self.pub_lidar_side_left.get_subscription_count()
            and self.pub_lidar_side_right.get_subscription_count()
            and self.pub_lidar_top.get_subscription_count()
            and self.count_subscribers("/sensing/lidar/concatenated/pointcloud") >= 3
        )

        centerpoint_ready = self.count_publishers(
            "/perception/object_recognition/detection/centerpoint/objects"
        )
        apollo_ready = self.count_publishers(
            "/perception/object_recognition/detection/apollo/labeled_clusters"
        )
        # vision_detection_ready = self.count_publishers(
        #     "/perception/object_recognition/detection/rois0"
        # )

        return lidar_subscribers_ready and (centerpoint_ready or apollo_ready)

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

        ground_truth_objects = DetectedObjects()
        ground_truth_objects.header.frame_id = "base_link"
        ground_truth_objects.header.stamp = ros_time.to_msg()

        for gt_object in current_scene["GT_OBJECTS"]:

            if gt_object.num_lidar_points_in_box <= 0:
                continue

            gt_detected_object = DetectedObject()
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

            ground_truth_objects.objects.append(gt_detected_object)

            o = metrics_pb2.Object()
            o.context_name = self.context_waymo
            o.frame_timestamp_micros = self.scene_timestamp

            box = label_pb2.Label.Box()
            box.center_x = gt_object.box.center_x
            box.center_y = gt_object.box.center_y
            box.center_z = gt_object.box.center_z
            box.length = gt_object.box.length
            box.width = gt_object.box.width
            box.height = gt_object.box.height
            box.heading = gt_object.box.heading
            o.object.box.CopyFrom(box)

            o.score = 1
            o.object.id = gt_object.id
            o.object.type = gt_object.type

            self.gt_proto_objects.objects.append(o)

        if self.is_dataset_finished():
            with open(self.gt_path, "ab+") as gt_file:
                gt_file.write(self.gt_proto_objects.SerializeToString())

        self.pub_gt_objects.publish(ground_truth_objects)

    def scene_processed(self):
        return self.current_scene_processed

    def set_scene_processed(self, value):
        self.current_scene_processed = value
