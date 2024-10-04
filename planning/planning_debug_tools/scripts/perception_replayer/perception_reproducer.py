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
from collections import deque
import pickle

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import StopWatch
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        self.rosbag_ego_odom_search_radius = args.search_radius
        self.ego_odom_search_radius = self.rosbag_ego_odom_search_radius
        self.reproduce_cool_down = args.reproduce_cool_down if args.search_radius != 0.0 else 0.0

        super().__init__(args, "perception_reproducer")

        self.stopwatch = StopWatch(self.args.verbose)  # for debug

        # refresh cool down for setting initial pose in psim.
        self.sub_init_pos = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", lambda msg: self.cool_down_indices.clear(), 1
        )

        # to make some data to accelerate computation
        self.preprocess_data()

        self.reproduce_sequence_indices = deque()  # contains ego_odom_idx
        self.cool_down_indices = deque()  # contains ego_odom_idx
        self.ego_odom_id2last_published_timestamp = {}  # for checking last published timestamp.
        self.last_sequenced_ego_pose = None

        pose_timestamp, self.prev_ego_odom_msg = self.rosbag_ego_odom_data[0]
        self.perv_objects_msg, self.prev_traffic_signals_msg = self.find_topics_by_timestamp(
            pose_timestamp
        )
        self.memorized_original_objects_msg = (
            self.memorized_noised_objects_msg
        ) = self.perv_objects_msg

        # start main timer callback

        average_ego_odom_interval = np.mean(
            [
                (self.rosbag_ego_odom_data[i][0] - self.rosbag_ego_odom_data[i - 1][0]) / 1e9
                for i in range(1, len(self.rosbag_ego_odom_data))
            ]
        )
        self.timer = self.create_timer(average_ego_odom_interval, self.on_timer)

        # kill perception process to avoid a conflict of the perception topics
        self.timer_check_perception_process = self.create_timer(3.0, self.on_timer_kill_perception)

        print("Start timer callback")

    def preprocess_data(self):
        # closest search with numpy data is much faster than usual
        self.rosbag_ego_odom_data_numpy = np.array(
            [
                [data[1].pose.pose.position.x, data[1].pose.pose.position.y]
                for data in self.rosbag_ego_odom_data
            ]
        )

    def on_timer_kill_perception(self):
        self.kill_online_perception_node()

    def on_timer(self):
        if self.args.verbose:
            print("\n-- on_timer start --")
        self.stopwatch.tic("total on_timer")

        timestamp = self.get_clock().now()
        timestamp_msg = timestamp.to_msg()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp_msg)
            self.pointcloud_pub.publish(pointcloud_msg)

        if not self.ego_odom:
            print("No ego odom found.")
            return

        ego_pose = self.ego_odom.pose.pose
        dist_moved = (
            np.sqrt(
                (ego_pose.position.x - self.last_sequenced_ego_pose.position.x) ** 2
                + (ego_pose.position.y - self.last_sequenced_ego_pose.position.y) ** 2
            )
            if self.last_sequenced_ego_pose
            else 999
        )

        # Update the reproduce sequence if the distance moved is greater than the search radius.
        if dist_moved > self.ego_odom_search_radius:
            self.last_sequenced_ego_pose = ego_pose

            # find the nearest ego odom by simulation observation
            self.stopwatch.tic("find_nearest_ego_odom_by_observation")
            nearby_ego_odom_indies = self.find_nearby_ego_odom_indies(
                [ego_pose], self.ego_odom_search_radius
            )
            nearby_ego_odom_indies = [
                self.rosbag_ego_odom_data[idx][1].pose.pose for idx in nearby_ego_odom_indies
            ]
            if not nearby_ego_odom_indies:
                nearest_ego_odom_ind = self.find_nearest_ego_odom_index(ego_pose)
                nearby_ego_odom_indies += [
                    self.rosbag_ego_odom_data[nearest_ego_odom_ind][1].pose.pose
                ]
            self.stopwatch.toc("find_nearest_ego_odom_by_observation")

            # find a list of ego odom around the nearest_ego_odom_pos.
            self.stopwatch.tic("find_nearby_ego_odom_indies")
            ego_odom_indices = self.find_nearby_ego_odom_indies(
                nearby_ego_odom_indies, self.rosbag_ego_odom_search_radius
            )
            self.stopwatch.toc("find_nearby_ego_odom_indies")

            # update reproduce_sequence with those data not in cool down list.
            while self.cool_down_indices:
                last_timestamp = self.ego_odom_id2last_published_timestamp[
                    self.cool_down_indices[0]
                ]
                if (timestamp - last_timestamp).nanoseconds / 1e9 > self.reproduce_cool_down:
                    self.cool_down_indices.popleft()
                else:
                    break

            self.stopwatch.tic("update reproduce_sequence")
            ego_odom_indices = [
                idx for idx in ego_odom_indices if idx not in self.cool_down_indices
            ]
            ego_odom_indices = sorted(ego_odom_indices)
            self.reproduce_sequence_indices = deque(ego_odom_indices)
            self.stopwatch.toc("update reproduce_sequence")

        if self.args.verbose:
            print("reproduce_sequence_indices: ", list(self.reproduce_sequence_indices)[:20])

        # Get messages
        repeat_flag = len(self.reproduce_sequence_indices) == 0

        # Add an additional constraint to avoid publishing too fast when there is a speed gap between the ego and the rosbag's ego when ego is departing/stopping while rosbag's ego is moving.
        if not repeat_flag:
            ego_speed = np.sqrt(
                self.ego_odom.twist.twist.linear.x**2 + self.ego_odom.twist.twist.linear.y**2
            )
            ego_odom_idx = self.reproduce_sequence_indices[0]
            _, ego_odom_msg = self.rosbag_ego_odom_data[ego_odom_idx]
            ego_rosbag_speed = np.sqrt(
                ego_odom_msg.twist.twist.linear.x**2 + ego_odom_msg.twist.twist.linear.y**2
            )

            ego_rosbag_dist = np.sqrt(
                (ego_pose.position.x - ego_odom_msg.pose.pose.position.x) ** 2
                + (ego_pose.position.y - ego_odom_msg.pose.pose.position.y) ** 2
            )
            repeat_flag = (
                ego_rosbag_speed > ego_speed * 2
                and ego_rosbag_speed > 3.0
                and ego_rosbag_dist > self.ego_odom_search_radius
            )
            # if ego_rosbag_speed is too fast than ego_speed, stop publishing the rosbag's ego odom message temporarily.

        if not repeat_flag:
            self.stopwatch.tic("find_topics_by_timestamp")
            ego_odom_idx = self.reproduce_sequence_indices.popleft()
            # extract messages by the nearest ego odom timestamp
            pose_timestamp, ego_odom_msg = self.rosbag_ego_odom_data[ego_odom_idx]
            objects_msg, traffic_signals_msg = self.find_topics_by_timestamp(pose_timestamp)
            self.stopwatch.toc("find_topics_by_timestamp")
            # update cool down info.
            self.ego_odom_id2last_published_timestamp[ego_odom_idx] = timestamp
            self.cool_down_indices.append(ego_odom_idx)
        else:
            ego_odom_msg = self.prev_ego_odom_msg
            objects_msg = self.perv_objects_msg
            traffic_signals_msg = self.prev_traffic_signals_msg

        # Transform and publish messages.
        self.stopwatch.tic("transform and publish")
        # ego odom
        if ego_odom_msg:
            self.prev_ego_odom_msg = ego_odom_msg
            self.recorded_ego_pub.publish(ego_odom_msg)
        # objects
        objects_msg = objects_msg if objects_msg else self.perv_objects_msg
        if objects_msg:
            self.perv_objects_msg = objects_msg
            objects_msg.header.stamp = timestamp_msg

            # add noise to repeat published objects
            if repeat_flag and self.args.noise:
                objects_msg = self.add_perception_noise(objects_msg)

            if self.args.detected_object:
                objects_msg = self.copy_message(objects_msg)
                translate_objects_coordinate(ego_pose, ego_odom_msg.pose.pose, objects_msg)

            self.objects_pub.publish(objects_msg)

        # traffic signals
        traffic_signals_msg = (
            traffic_signals_msg if traffic_signals_msg else self.prev_traffic_signals_msg
        )
        if traffic_signals_msg:
            traffic_signals_msg.stamp = timestamp_msg
            self.prev_traffic_signals_msg = traffic_signals_msg
            self.traffic_signals_pub.publish(traffic_signals_msg)

        self.stopwatch.toc("transform and publish")

        self.stopwatch.toc("total on_timer")

    def find_nearest_ego_odom_index(self, ego_pose):
        # nearest search with numpy format is much (~ x100) faster than regular for loop
        self_pose = np.array([ego_pose.position.x, ego_pose.position.y])
        dists_squared = np.sum((self.rosbag_ego_odom_data_numpy - self_pose) ** 2, axis=1)
        nearest_idx = np.argmin(dists_squared)
        return nearest_idx

    def find_nearby_ego_odom_indies(self, ego_poses: list, search_radius: float):
        ego_poses_np = np.array([[pose.position.x, pose.position.y] for pose in ego_poses])
        dists_squared = np.sum(
            (self.rosbag_ego_odom_data_numpy[:, None] - ego_poses_np) ** 2, axis=2
        )
        nearby_indices = np.where(np.any(dists_squared <= search_radius**2, axis=1))[0]

        return nearby_indices

    def copy_message(self, msg):
        self.stopwatch.tic("message deepcopy")
        objects_msg_copied = pickle.loads(pickle.dumps(msg))  # this is x5 faster than deepcopy
        self.stopwatch.toc("message deepcopy")
        return objects_msg_copied

    def add_perception_noise(
        self, objects_msg, update_rate=0.03, x_noise_std=0.1, y_noise_std=0.05
    ):
        if self.memorized_original_objects_msg != objects_msg:
            self.memorized_noised_objects_msg = self.memorized_original_objects_msg = objects_msg

        if np.random.rand() < update_rate:
            self.stopwatch.tic("add noise")
            self.memorized_noised_objects_msg = self.copy_message(
                self.memorized_original_objects_msg
            )
            for obj in self.memorized_noised_objects_msg.objects:
                noise_x = np.random.normal(0, x_noise_std)
                noise_y = np.random.normal(0, y_noise_std)
                if self.args.detected_object or self.args.tracked_object:
                    obj.kinematics.pose_with_covariance.pose.position.x += noise_x
                    obj.kinematics.pose_with_covariance.pose.position.y += noise_y
                else:
                    obj.kinematics.initial_pose_with_covariance.pose.position.x += noise_x
                    obj.kinematics.initial_pose_with_covariance.pose.position.y += noise_y
            self.stopwatch.toc("add noise")

        return self.memorized_noised_objects_msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag", help="rosbag", default=None)
    parser.add_argument(
        "-n",
        "--noise",
        help="apply perception noise to the objects when publishing repeated messages",
        action="store_true",
        default=True,
    )
    parser.add_argument(
        "-d", "--detected-object", help="publish detected object", action="store_true"
    )
    parser.add_argument(
        "-t", "--tracked-object", help="publish tracked object", action="store_true"
    )
    parser.add_argument(
        "-f", "--rosbag-format", help="rosbag data format (default is db3)", default="db3"
    )
    parser.add_argument(
        "-v", "--verbose", help="output debug data", action="store_true", default=False
    )
    parser.add_argument(
        "-r",
        "--search-radius",
        help="the search radius for searching rosbag's ego odom messages around the nearest ego odom pose (default is 1.5 meters), if the search radius is set to 0, it will always publish the closest message, just as the old reproducer did.",
        type=float,
        default=1.5,
    )
    parser.add_argument(
        "-c",
        "--reproduce-cool-down",
        help="The cool down time for republishing published messages (default is 80.0 seconds), please make sure that it's greater than the ego's stopping time.",
        type=float,
        default=80.0,
    )

    args = parser.parse_args()

    rclpy.init()
    node = PerceptionReproducer(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
