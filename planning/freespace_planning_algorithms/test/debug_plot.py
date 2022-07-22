#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc.
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

from dataclasses import dataclass
from math import asin
from math import atan2
from math import cos
from math import sin
import os
from typing import Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64


@dataclass
class ProblemDescription:
    costmap: OccupancyGrid
    start: Pose
    goal: Pose
    trajectory: PoseArray
    vehicle_length: Float64
    vehicle_width: Float64
    vehicle_base2back: Float64
    elapsed_time: Float64

    @classmethod
    def from_rosbag_path(cls, path: str) -> "ProblemDescription":
        # ref: rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options, converter_options = cls.get_rosbag_options(bag_path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()

        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        message_map = {}

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            message_map[topic] = msg

        return cls(**message_map)

    @staticmethod
    def get_rosbag_options(path: str, serialization_format="cdr"):
        # copied from rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )

        return storage_options, converter_options


@dataclass
class VehicleModel:
    length: float
    width: float
    base2back: float

    @classmethod
    def from_problem_description(cls, pd: ProblemDescription) -> "VehicleModel":
        return cls(pd.vehicle_length.data, pd.vehicle_width.data, pd.vehicle_base2back.data)

    def get_vertices(self, pose: Pose) -> np.ndarray:
        x, y, yaw = self.posemsg_to_nparr(pose)

        back = -1.0 * self.base2back
        front = self.length - self.base2back
        right = -0.5 * self.width
        left = 0.5 * self.width
        vertices_local = np.array([[back, left], [back, right], [front, right], [front, left]])

        R_mat = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        vertices_global = vertices_local.dot(R_mat.T) + np.array([x, y])
        return vertices_global

    def plot_pose(self, pose: Pose, ax, color="black", lw=1):
        x = pose.position.x
        y = pose.position.y
        V = self.get_vertices(pose)
        ax.scatter(x, y, c=color, s=2)
        for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
            i, j = idx_pair
            ax.plot([V[i, 0], V[j, 0]], [V[i, 1], V[j, 1]], color=color, linewidth=lw)

    @staticmethod
    def euler_from_quaternion(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def posemsg_to_nparr(pose_msg: Pose) -> Tuple[float, float, float]:
        _, _, yaw = VehicleModel.euler_from_quaternion(pose_msg.orientation)
        return pose_msg.position.x, pose_msg.position.y, yaw


def plot_problem(pd: ProblemDescription, ax):
    info = pd.costmap.info
    n_grid = np.array([info.width, info.height])
    res = info.resolution
    origin = info.origin
    arr = np.array(pd.costmap.data).reshape((n_grid[1], n_grid[0]))
    b_min = np.array([origin.position.x, origin.position.y])
    b_max = b_min + n_grid * res

    x_lin, y_lin = [np.linspace(b_min[i], b_max[i], n_grid[i]) for i in range(2)]
    X, Y = np.meshgrid(x_lin, y_lin)
    ax.contourf(X, Y, arr, cmap="Greys")

    vmodel = VehicleModel.from_problem_description(pd)
    vmodel.plot_pose(pd.start, ax, "green")
    vmodel.plot_pose(pd.goal, ax, "red")

    for pose in pd.trajectory.poses:
        vmodel.plot_pose(pose, ax, "black", 0.5)

    text = "elapsed : {0} [msec]".format(int(round(pd.elapsed_time.data)))
    ax.text(0.3, 0.3, text, fontsize=15, color="gray")

    ax.set_xlim([0, 10])
    ax.set_ylim([b_min[1], b_max[1]])
    ax.axis("equal")


if __name__ == "__main__":
    for cand_dir in os.listdir("/tmp"):
        if cand_dir.startswith("fpalgos"):
            bag_path = os.path.join("/tmp", cand_dir)
            pd = ProblemDescription.from_rosbag_path(bag_path)

            fig, ax = plt.subplots()
            plot_problem(pd, ax)
            fig.tight_layout()

            file_name = os.path.join("/tmp", cand_dir + "-plot.png")
            plt.savefig(file_name)
            print("saved to {}".format(file_name))
