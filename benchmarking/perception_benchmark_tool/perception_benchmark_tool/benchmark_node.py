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

import os

from autoware_auto_perception_msgs.msg import ObjectClassification
from autoware_auto_perception_msgs.msg import TrackedObjects
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber
from message_filters import TimeSynchronizer
from perception_benchmark_tool.benchmark_tools.math_utils import euler_from_quaternion
from perception_benchmark_tool.benchmark_tools.ros_utils import do_transform_pose_stamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from waymo_open_dataset import label_pb2
from waymo_open_dataset.protos import metrics_pb2


def generate_context_name():
    generate_context_name.__dict__.setdefault("count", 0)
    generate_context_name.count += 1
    return "context_" + str(generate_context_name.count)


class PerceptionBenchmark(Node):
    def __init__(self):
        super().__init__("benchmark_node")

        self.declare_parameter("benchmark_frame", "base_link")
        self.benchmark_frame = (
            self.get_parameter("benchmark_frame").get_parameter_value().string_value
        )

        self.declare_parameter("result_path", "")
        self.result_path = self.get_parameter("result_path").get_parameter_value().string_value

        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)

        self.prediction_result_path = self.result_path + "/prediction_result.bin"
        self.gt_path = self.result_path + "/ground_truth_result.bin"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.sub_tracking = Subscriber(
            self, TrackedObjects, "/perception/object_recognition/tracking/objects"
        )

        self.sub_tracking_gt = Subscriber(self, TrackedObjects, "/gt_objects")
        self.time_sync = TimeSynchronizer([self.sub_tracking, self.sub_tracking_gt], 10)
        self.time_sync.registerCallback(self.tracking_benchmark_callback)

        self.sub_scene_finished = self.create_subscription(
            Bool, "segment_finished", self.scene_changed_callback, 1
        )

        self.context_name = generate_context_name()

    def tracking_benchmark_callback(self, tracking_objects, gt_tracking_objects):

        prediction_proto_objects = metrics_pb2.Objects()
        gt_proto_objects = metrics_pb2.Objects()

        objects_stamp = tracking_objects.header.stamp
        waymo_timestamp_as_micro = objects_stamp.sec * 1000000 + objects_stamp.nanosec / 1000

        if tracking_objects.header.frame_id != self.benchmark_frame:
            tracking_objects = self.transform_tracked_objects(tracking_objects)

        # Convert tracked obejct to Waymo Proto objects
        for tracked_object in tracking_objects.objects:

            tracked_object_waymo = metrics_pb2.Object()
            tracked_object_waymo.context_name = self.context_name
            tracked_object_waymo.frame_timestamp_micros = int(waymo_timestamp_as_micro)

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

            prediction_proto_objects.objects.append(tracked_object_waymo)

        with open(self.prediction_result_path, "ab+") as pred_file:
            pred_file.write(prediction_proto_objects.SerializeToString())

        if gt_tracking_objects.header.frame_id != self.benchmark_frame:
            gt_tracking_objects = self.transform_tracked_objects(tracking_objects)

        # Convert ground truth obejct to Waymo Proto objects
        for gt_object in gt_tracking_objects.objects:

            gt_waymo = metrics_pb2.Object()
            gt_waymo.context_name = self.context_name
            gt_waymo.frame_timestamp_micros = int(waymo_timestamp_as_micro)

            bbox_waymo = label_pb2.Label.Box()
            bbox_waymo.center_x = gt_object.kinematics.pose_with_covariance.pose.position.x
            bbox_waymo.center_y = gt_object.kinematics.pose_with_covariance.pose.position.y
            bbox_waymo.center_z = gt_object.kinematics.pose_with_covariance.pose.position.z
            bbox_waymo.length = gt_object.shape.dimensions.x
            bbox_waymo.width = gt_object.shape.dimensions.y
            bbox_waymo.height = gt_object.shape.dimensions.z

            roll, pitch, yaw = euler_from_quaternion(
                gt_object.kinematics.pose_with_covariance.pose.orientation.x,
                gt_object.kinematics.pose_with_covariance.pose.orientation.y,
                gt_object.kinematics.pose_with_covariance.pose.orientation.z,
                gt_object.kinematics.pose_with_covariance.pose.orientation.w,
            )

            bbox_waymo.heading = yaw
            gt_waymo.object.box.CopyFrom(bbox_waymo)
            gt_waymo.score = 0.5
            if gt_object.classification[0].label == ObjectClassification.CAR:
                gt_waymo.object.type = label_pb2.Label.TYPE_VEHICLE
            elif gt_object.classification[0].label == ObjectClassification.PEDESTRIAN:
                gt_waymo.object.type = label_pb2.Label.TYPE_PEDESTRIAN
            elif gt_object.classification[0].label == ObjectClassification.BICYCLE:
                gt_waymo.object.type = label_pb2.Label.TYPE_CYCLIST
            else:
                continue

            object_track_id = "".join(
                str(gt_object.object_id.uuid[e]) for e in range(0, len(gt_object.object_id.uuid))
            )
            gt_waymo.object.id = str(object_track_id)

            gt_proto_objects.objects.append(gt_waymo)

        with open(self.gt_path, "ab+") as gt_file:
            gt_file.write(gt_proto_objects.SerializeToString())

    def transform_tracked_objects(self, tracked_objects):

        transformed_tracked_objects = TrackedObjects()
        transformed_tracked_objects.header.stamp = tracked_objects.header.stamp
        transformed_tracked_objects.header.frame_id = self.benchmark_frame

        try:
            trans = self.tf_buffer.lookup_transform(
                self.benchmark_frame,
                tracked_objects.header.frame_id,
                tracked_objects.header.stamp,
                rclpy.duration.Duration(seconds=1),
            )

        except TransformException as ex:
            self.get_logger().info("Could not find transform:" + str(ex))
            return None

        for tracked_object in tracked_objects.objects:
            tracked_objected_map_pose = PoseStamped()
            tracked_objected_map_pose.header.stamp = tracked_objects.header.stamp
            tracked_objected_map_pose.header.frame_id = tracked_objects.header.frame_id
            tracked_objected_map_pose.pose.position = (
                tracked_object.kinematics.pose_with_covariance.pose.position
            )
            tracked_objected_map_pose.pose.orientation = (
                tracked_object.kinematics.pose_with_covariance.pose.orientation
            )

            object_transformed = do_transform_pose_stamped(tracked_objected_map_pose, trans)

            transformed_tracked_object = tracked_object
            transformed_tracked_object.kinematics.pose_with_covariance.pose.orientation = (
                object_transformed.pose.orientation
            )
            transformed_tracked_object.kinematics.pose_with_covariance.pose.position = (
                object_transformed.pose.position
            )

            transformed_tracked_objects.objects.append(transformed_tracked_object)

        return transformed_tracked_objects

    def scene_changed_callback(self, read_dataset):
        self.context_name = generate_context_name()
        self.tf_buffer.clear()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)


def main(args=None):
    rclpy.init(args=args)
    benchmark_node = PerceptionBenchmark()
    rclpy.spin(benchmark_node)
    rclpy.shutdown()
