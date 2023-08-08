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

import argparse
import copy
import math

from nav_msgs.msg import Odometry
import numpy as np
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import calc_squared_distance
from utils import create_empty_pointcloud
from utils import get_quaternion_from_yaw
from utils import get_yaw_from_quaternion


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        super().__init__(args, "perception_reproducer")

        # additional subscriber
        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odom, 1
        )

        self.ego_pose_idx = None
        self.ego_pose = None
        self.prev_traffic_signals_msg = None

        # start timer callback
        self.timer = self.create_timer(0.01, self.on_timer)
        print("Start timer callback")

    def on_odom(self, odom):
        self.ego_pose = odom.pose.pose

    def on_timer(self):
        timestamp = self.get_clock().now().to_msg()

        self.kill_online_perception_node()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp)
            self.pointcloud_pub.publish(pointcloud_msg)

        if self.ego_pose:
            ego_odom = self.find_nearest_ego_odom_by_observation(self.ego_pose)
            pose_timestamp = ego_odom[0]
            log_ego_pose = ego_odom[1].pose.pose

            msgs = copy.deepcopy(self.find_topics_by_timestamp(pose_timestamp))
            objects_msg = msgs[0]
            traffic_signals_msg = msgs[1]
            if objects_msg:
                objects_msg.header.stamp = timestamp
                if self.args.detected_object:
                    log_ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation)
                    log_ego_pose_trans_mat = np.array(
                        [
                            [
                                math.cos(log_ego_yaw),
                                -math.sin(log_ego_yaw),
                                log_ego_pose.position.x,
                            ],
                            [math.sin(log_ego_yaw), math.cos(log_ego_yaw), log_ego_pose.position.y],
                            [0.0, 0.0, 1.0],
                        ]
                    )

                    ego_yaw = get_yaw_from_quaternion(self.ego_pose.orientation)
                    ego_pose_trans_mat = np.array(
                        [
                            [math.cos(ego_yaw), -math.sin(ego_yaw), self.ego_pose.position.x],
                            [math.sin(ego_yaw), math.cos(ego_yaw), self.ego_pose.position.y],
                            [0.0, 0.0, 1.0],
                        ]
                    )

                    for o in objects_msg.objects:
                        log_object_pose = o.kinematics.pose_with_covariance.pose
                        log_object_yaw = get_yaw_from_quaternion(log_object_pose.orientation)
                        log_object_pos_vec = np.array(
                            [log_object_pose.position.x, log_object_pose.position.y, 1.0]
                        )

                        # translate object pose from ego pose in log to ego pose in simulation
                        object_pos_vec = np.linalg.inv(ego_pose_trans_mat).dot(
                            log_ego_pose_trans_mat.dot(log_object_pos_vec.T)
                        )

                        object_pose = o.kinematics.pose_with_covariance.pose
                        object_pose.position.x = object_pos_vec[0]
                        object_pose.position.y = object_pos_vec[1]
                        object_pose.orientation = get_quaternion_from_yaw(
                            log_object_yaw + log_ego_yaw - ego_yaw
                        )

                self.objects_pub.publish(objects_msg)
            if traffic_signals_msg:
                traffic_signals_msg.header.stamp = timestamp
                self.traffic_signals_pub.publish(traffic_signals_msg)
                self.prev_traffic_signals_msg = traffic_signals_msg
            elif self.prev_traffic_signals_msg:
                self.prev_traffic_signals_msg.header.stamp = timestamp
                self.traffic_signals_pub.publish(self.prev_traffic_signals_msg)
        else:
            print("No ego pose found.")

    def find_nearest_ego_odom_by_observation(self, ego_pose):
        if self.ego_pose_idx:
            start_idx = self.ego_pose_idx - 10
            end_idx = self.ego_pose_idx + 10
        else:
            start_idx = 0
            end_idx = len(self.rosbag_ego_odom_data) - 1

        nearest_idx = 0
        nearest_dist = float("inf")
        for idx in range(start_idx, end_idx + 1):
            data = self.rosbag_ego_odom_data[idx]
            dist = calc_squared_distance(data[1].pose.pose.position, ego_pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = idx

        return self.rosbag_ego_odom_data[nearest_idx]

    def find_topics_by_timestamp(self, timestamp):
        objects_data = None
        for data in self.rosbag_objects_data:
            if timestamp < data[0]:
                objects_data = data[1]
                break

        traffic_signals_data = None
        for data in self.rosbag_traffic_signals_data:
            if timestamp < data[0]:
                traffic_signals_data = data[1]
                break

        return objects_data, traffic_signals_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag", help="rosbag", default=None)
    parser.add_argument(
        "-d", "--detected-object", help="publish detected object", action="store_true"
    )
    parser.add_argument(
        "-t", "--tracked-object", help="publish tracked object", action="store_true"
    )
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionReproducer(args)
    rclpy.spin(node)

    try:
        rclpy.init()
        node = PerceptionReproducer(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
