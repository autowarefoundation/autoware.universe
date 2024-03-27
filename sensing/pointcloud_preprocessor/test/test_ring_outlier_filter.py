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

import unittest

import copy
from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
from launch.logging import get_logger
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np
import pytest
import random
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter_node",
            parameters=[
                {"use_imu": False},
            ],
            remappings=[
                ("input", "/test/sensing/lidar/top/mirror_cropped/pointcloud_ex"),
                # ("~/input/indices", "/test/sensing/lidar/top/mirror_cropped/pointcloud_ex"),
                ("output", "/test/sensing/lidar/top/rectified/pointcloud_ex"),
            ],
        )
    )

    container = ComposableNodeContainer(
        name="test_ring_outlier_filter_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRingOutlierFilter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.test_node = rclpy.create_node("test_node")
        # self.evaluation_time = 2  # 200ms ??
        self.evaluation_time = 100

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    @staticmethod
    def pointcloud2_to_xyz_array(cloud_msg):
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)

        print("-----------------cloud_arr-----------------")
        print(cloud_arr)
        print("-----------------cloud_arr-----------------")
        
        cloud_arr = np.reshape(cloud_arr, (cloud_msg.width, int(cloud_msg.point_step / 4)))
        return cloud_arr[:, :3]

    @staticmethod
    def generate_pointcloud(header, timestamp_offset_per_point=0.01):
         
        def generate_points_on_circle(N, r, Z, diffusion_N):
            theta = np.linspace(0, 2*np.pi, N, endpoint=False)
            distance = np.full(N, r)

            x = r * np.cos(theta).astype(np.float32)
            y = r * np.sin(theta).astype(np.float32)
            z = np.full_like(x, Z).astype(np.float32)
            
            idx_list = sorted(random.sample(range(0, N), k=diffusion_N), reverse=True)
            
            for idx in idx_list:
                diffusion_r = random.uniform(0.1, r-0.1)
                x[idx] = diffusion_r * np.cos(theta[idx]).astype(np.float32)
                y[idx] = diffusion_r * np.sin(theta[idx]).astype(np.float32)
                distance[idx] = np.float32(diffusion_r)
                    
            points = np.column_stack((x, y, z))
            
            # eliminate outlier points
            gt_points = np.delete(points, idx_list, 0)
            gt_theta = np.delete(theta, idx_list, 0) # for azimuth
            gt_distance = np.delete(distance, idx_list, 0) # for distance


            print("------------points------------")
            print("points: ", points)
            print("gt_points: ", gt_points)
            print("------------points------------")
            
            return points, theta, distance, gt_points, gt_theta, gt_distance

        num_points = 10 # the number of division in a circle line
        num_points_diffusion = 5
        points_xyz, theta, distance, gt_points_xyz, gt_theta, gt_distance = generate_points_on_circle(num_points, 3.0, 2.0, num_points_diffusion)
        
        distance = np.asarray([np.float32(d) for d in distance])
        gt_distance = np.asarray([np.float32(d) for d in gt_distance])
        
        # print("x--------------------------------x")
        # print("points_xyz: ", len(points_xyz))
        # print("x--------------------------------x")
        azimuth = [450-(th/np.pi)*180 for th in theta]
        azimuth = np.array([np.float32(az-360) if az>=360 else np.float32(az) for az in azimuth])
        timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + timestamp_offset_per_point * i
                for i in range(num_points)
            ]
        ).astype(np.float64)
        xyz_data = points_xyz.tobytes()
        timestamp_data = timestamps.tobytes()
        distance_data = distance.tobytes()
        azimuth_data = azimuth.tobytes()
        
        intensity = np.full(num_points, np.float32('10.0'))
        ring = np.full(num_points, np.int16('42'))
        intensity_data = intensity.tobytes()
        ring_data = ring.tobytes()
        
        # print("x--------------------------------x")
        # print("xyz_data: ", len(xyz_data))
        # print("x--------------------------------x")
        # pointcloud_data = b"".join(
        #     xyz_data[i * 12 : i * 12 + 12] + timestamp_data[i * 8 : i * 8 + 8]
        #     for i in range(len(xyz_data))
        # )

        tmp = [xyz_data[i * 12 : i * 12 + 12] + intensity_data[i * 4 : i * 4 + 4] + ring_data[i * 2 : i * 2 + 2] + azimuth_data[i * 4 : i * 4 + 4] + distance_data[i * 4 : i * 4 + 4] +  timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(xyz_data))]
        # print("------------------------aa------------------------")
        # print("tmp: ", tmp[0])
        # print("len(tmp): ", len(tmp[0])) # 34
        # print("len(xyz_data): ", len(xyz_data)) # 120
        # print("intensity: ", intensity)
        # print("len(intensity_data): ", len(intensity_data)) # 40
        # print("len(ring_data): ", len(ring_data)) # 20
        # print("len(timestamp_data): ", len(timestamp_data)) # 80
        # print("len(distance_data): ", len(distance_data)) # 40
        # print("len(azimuth_data): ", len(azimuth_data)) # 40


        # print("------------------------aa------------------------")
        pointcloud_data = b"".join(
            xyz_data[i * 12 : i * 12 + 12] + intensity_data[i * 4 : i * 4 + 4] + ring_data[i * 2 : i * 2 + 2] + azimuth_data[i * 4 : i * 4 + 4] + distance_data[i * 4 : i * 4 + 4] +  timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(xyz_data))
        )
        # print("pointcloud_data: ", len(pointcloud_data))
        # print("------------------------aa------------------------")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="azimuth", offset=18, datatype=PointField.FLOAT32, count=1),
            PointField(name="distance", offset=22, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_stamp", offset=26, datatype=PointField.FLOAT64, count=1),
        ]
        # print("x--------------------------------x")
        # print("pointcloud_data: ", len(pointcloud_data))
        # print("x--------------------------------x")
        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=num_points,
            is_dense=True,
            is_bigendian=False,
            point_step=34,
            row_step=34 * num_points,  # point_step * width
            fields=fields,
            data=pointcloud_data,
        )

        # ground_truth_pointcloud
        gt_num_points = num_points - num_points_diffusion
        gt_azimuth = [450-(th/np.pi)*180 for th in gt_theta]
        gt_azimuth = np.array([np.float32(az-360) if az>=360 else np.float32(az) for az in gt_azimuth])
        gt_timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + timestamp_offset_per_point * i
                for i in range(gt_num_points)
            ]
        ).astype(np.float64)
        gt_xyz_data = gt_points_xyz.tobytes()
        gt_timestamp_data = gt_timestamps.tobytes()
        gt_distance_data = gt_distance.tobytes()
        gt_azimuth_data = gt_azimuth.tobytes()
        
        # gt_pointcloud_data = b"".join(
        #     gt_xyz_data[i * 12 : i * 12 + 12] + gt_timestamp_data[i * 8 : i * 8 + 8]
        #     for i in range(len(gt_xyz_data))
        # )
        # gt_fields = [
        #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
        # ]
        gt_tmp = [gt_xyz_data[i * 12 : i * 12 + 12] + intensity_data[i * 4 : i * 4 + 4] + ring_data[i * 2 : i * 2 + 2] + gt_azimuth_data[i * 4 : i * 4 + 4] + gt_distance_data[i * 4 : i * 4 + 4] + gt_timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(gt_xyz_data))]
        # print("------------------------aa------------------------")
        # print("gt_tmp: ", gt_tmp[0])
        # print("gt_len(tmp): ", len(gt_tmp[0]))
        # print("------------------------aa------------------------")
        
        gt_pointcloud_data = b"".join(
            gt_xyz_data[i * 12 : i * 12 + 12] + intensity_data[i * 4 : i * 4 + 4] + ring_data[i * 2 : i * 2 + 2] + gt_azimuth_data[i * 4 : i * 4 + 4] + gt_distance_data[i * 4 : i * 4 + 4] + gt_timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(gt_xyz_data))
        )
        # print("gt_pointcloud_data: ", len(gt_pointcloud_data))
        # print("------------------------aa------------------------")
        gt_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="azimuth", offset=18, datatype=PointField.FLOAT32, count=1),
            PointField(name="distance", offset=22, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_stamp", offset=26, datatype=PointField.FLOAT64, count=1),
        ]
        gt_pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=num_points-num_points_diffusion,
            is_dense=True,
            is_bigendian=False,
            point_step=34,
            row_step=34 * gt_num_points,  # point_step * width
            fields=gt_fields,
            data=gt_pointcloud_data,
        )
        return pointcloud_msg, gt_pointcloud_msg

    @staticmethod
    def stale_generate_pointcloud(num_points, header, timestamp_offset_per_point=0.01):
        points_xyz = np.random.rand(num_points, 3).astype(np.float32)
        timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + timestamp_offset_per_point * i
                for i in range(num_points)
            ]
        ).astype(np.float64)
        xyz_data = points_xyz.tobytes()
        timestamp_data = timestamps.tobytes()
        pointcloud_data = b"".join(
            xyz_data[i * 12 : i * 12 + 12] + timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(xyz_data))
        )
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
        ]
        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=10,
            is_dense=True,
            is_bigendian=False,
            point_step=20,  # 4 float32 fields * 4 bytes/field
            row_step=20 * num_points,  # point_step * width
            fields=fields,
            data=pointcloud_data,
        )
        gt_pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=10,
            is_dense=True,
            is_bigendian=False,
            point_step=20,  # 4 float32 fields * 4 bytes/field
            row_step=20 * num_points,  # point_step * width
            fields=fields,
            data=pointcloud_data,
        )
        return pointcloud_msg, gt_pointcloud_msg

    def test_node_link(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publishers
        pointcloud_pub = self.test_node.create_publisher(
            PointCloud2,
            "/test/sensing/lidar/top/mirror_cropped/pointcloud_ex",
            qos_profile=sensor_qos,
        )

        # Subscribers
        received_data = {"msg": None}

        def pointcloud_callback(msg):
            received_data["msg"] = msg

        self.test_node.create_subscription(
            PointCloud2,
            "/test/sensing/lidar/top/rectified/pointcloud_ex",
            pointcloud_callback,
            qos_profile=sensor_qos,
        )

        # Wait for composable node launches
        rclpy.spin_once(self.test_node, timeout_sec=1.0)

        # Prepare header
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Publish pointcloud data
        num_points = 10
        pointcloud_input_msg, pointcloud_ground_truth_msg = self.generate_pointcloud(header)
        # pointcloud_input_msg, pointcloud_ground_truth_msg = self.stale_generate_pointcloud(num_points, header)
        print(":---------------------------------------------:")
        print("pointcloud_input_msg: ", pointcloud_input_msg)
        print(":---------------------------------------------:")
        print("pointcloud_ground_truth_msg: ", pointcloud_ground_truth_msg)
        print(":---------------------------------------------:")
        # pointcloud_ground_truth_msg = self.generate_pointcloud(header, ground_truth=True)
        pointcloud_pub.publish(pointcloud_input_msg)

        # Wait for output with a timeout
        start_time = self.test_node.get_clock().now()
        while self.test_node.get_clock().now() - start_time < rclpy.duration.Duration(
            seconds=self.evaluation_time
        ):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if received_data["msg"] is not None:
                print("MESSAGE RECEIVED")
                break

        print(":---------------------------------------------:")
        print("received_data: ", received_data["msg"])
        print(":---------------------------------------------:")
        # Check if the output was received
        self.assertIsNotNone(received_data["msg"], "Did not receive output pointcloud data")

        # Check that the received pointcloud data has same length as num_points
        received_pointcloud = received_data["msg"]
        # self.assertEqual(
        #     received_pointcloud.width,
        #     num_points,
        #     "The received pointcloud data has a different length than expected",
        # )

        # Check that the received pointcloud data is different from the original one
        # original_pointcloud_arr = self.pointcloud2_to_xyz_array(pointcloud_input_msg)
        gt_pointcloud_arr = self.pointcloud2_to_xyz_array(pointcloud_ground_truth_msg)
        received_pointcloud_arr = self.pointcloud2_to_xyz_array(received_pointcloud)
        self.assertFalse(
            np.allclose(gt_pointcloud_arr, received_pointcloud_arr, atol=1e-6),
            "The received pointcloud data is not different from the original one",
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)