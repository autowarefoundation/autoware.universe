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

import math

from geometry_msgs.msg import Quaternion
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def open_reader(path: str):
    storage_options, converter_options = get_rosbag_options(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def calc_squared_distance(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def create_empty_pointcloud(timestamp):
    pointcloud_msg = PointCloud2()
    pointcloud_msg.header.stamp = timestamp
    pointcloud_msg.header.frame_id = "map"
    pointcloud_msg.height = 1
    pointcloud_msg.is_dense = True
    pointcloud_msg.point_step = 16
    field_name_vec = ["x", "y", "z"]
    offset_vec = [0, 4, 8]
    for field_name, offset in zip(field_name_vec, offset_vec):
        field = PointField()
        field.name = field_name
        field.offset = offset
        field.datatype = 7
        field.count = 1
        pointcloud_msg.fields.append(field)
    return pointcloud_msg


def get_yaw_from_quaternion(orientation):
    orientation_list = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]
    return euler_from_quaternion(orientation_list)[2]


def get_quaternion_from_yaw(yaw):
    q = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion()
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    return orientation
