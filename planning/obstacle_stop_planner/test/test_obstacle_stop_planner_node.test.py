# Copyright 2022 Tier IV, Inc.
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

from dataclasses import field
import imp
from operator import truediv
import time
import unittest
import std_msgs.msg
import sensor_msgs.msg
import autoware_auto_planning_msgs.msg
import autoware_auto_perception_msgs.msg
import nav_msgs.msg
import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
import launch_testing
import pytest
import rclpy
from rclpy.node import Node
import copy


@pytest.mark.launch_test

def generate_test_description():
    
    print("==================== launch test start =====================")
    obstacle_stop_planner_node = launch_ros.actions.Node(
        package='obstacle_stop_planner',
        executable='obstacle_stop_planner_node',
        name='obstacle_stop_planner_node',
        remappings=[
            ('~/input/pointcloud', 'input/pointcloud'),
            ('~/input/trajectory', 'input/trajectory'),
            ('~/input/odometry', 'input/odometry'),
            ('~/input/objects', 'input/objects'),
            ('~/output/trajectory', 'output/trajectory')
        ],
        parameters=[
            [FindPackageShare('vehicle_info_util'), '/config/vehicle_info.param.yaml'],
            [FindPackageShare('obstacle_stop_planner'), '/config/obstacle_stop_planner.param.yaml'],
            [FindPackageShare('obstacle_stop_planner'), '/config/common.param.yaml'],
            [FindPackageShare('obstacle_stop_planner'), '/config/adaptive_cruise_control.param.yaml']
        ],
    )
    static_transform_publisher_node = launch_ros.actions.Node(package = "tf2_ros", 
        executable = "static_transform_publisher",
        name='static_transform_publisher_node',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "base_link"])

    return (
        launch.LaunchDescription([
            obstacle_stop_planner_node,
            static_transform_publisher_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'obstacle_stop_planner': obstacle_stop_planner_node,
            'static_transform_publisher': static_transform_publisher_node
        }
    )


class TestObstacleStopPlannerLink(unittest.TestCase):

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
        self.node = rclpy.create_node('test_object_stop_planner_link')
        self.event_name = 'test_object_stop_planner'

    def tearDown(self):
        self.node.destroy_node()

    """
    
    def test_simple_calc(self):
        calc_result=5+5
        self.assertEqual(10,calc_result) 

    def test_simple_calc2(self):
        calc_result=5+5
        self.assertEqual(10,calc_result)
    
    def test_pubsub(self):
        rx_data=[]

        #publisher
        pub = self.node.create_publisher(
            std_msgs.msg.Int32,
            'test_data',
            10
        )

        #subscliber
        sub = self.node.create_subscription(
            std_msgs.msg.Int32,
            'test_data',
            lambda msg: rx_data.append(msg),
            10
        )

        try:
            end_time = time.time() + 10

            tx_data=std_msgs.msg.Int32()
            tx_data.data=10
            pub.publish(tx_data)
            #timeout 10[s]
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if len(rx_data)>0:
                    break
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)
            self.assertEqual(tx_data.data,rx_data[0].data)
    """
    

    def trajectory_callback(self,msg):
        print("-------trajectory_callback------")

    def test_linear_trajectory(self):
        #publisher
        pointcloud_pub = self.node.create_publisher(sensor_msgs.msg.PointCloud2, 'input/pointcloud', 10)
        trajectory_pub = self.node.create_publisher(autoware_auto_planning_msgs.msg.Trajectory, 'input/trajectory', 10)
        odom_pub = self.node.create_publisher(nav_msgs.msg.Odometry, 'input/odometry', 10)
        object_pub = self.node.create_publisher(autoware_auto_perception_msgs.msg.PredictedObjects, 'input/objects', 10)

        #subscliber
        trajectory_sub = self.node.create_subscription(autoware_auto_planning_msgs.msg.Trajectory, 'output/trajectory', self.trajectory_callback, 10)
        
        #param
        map_frame="map"
        base_link_frame="base_link"

        #object
        object_msg=autoware_auto_perception_msgs.msg.PredictedObjects()
        object_msg.header.frame_id=map_frame
        object_msg.header.stamp=self.node.get_clock().now().to_msg()
        object_pub.publish(object_msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        while rclpy.ok():

            #message init
            #pointcloud2 
            pointcloud_msg=sensor_msgs.msg.PointCloud2()
            pointcloud_msg.header.frame_id=base_link_frame
            pointcloud_msg.header.stamp=self.node.get_clock().now().to_msg()
            pointcloud_msg.height=1
            pointcloud_msg.width=0
            field=sensor_msgs.msg.PointField()
            field.name="x"
            field.offset=0
            field.datatype=7
            field.count=1
            pointcloud_msg.fields.append(copy.deepcopy(field))
            field.name="y"
            field.offset=4
            pointcloud_msg.fields.append(copy.deepcopy(field))
            field.name="z"
            field.offset=8
            pointcloud_msg.fields.append(copy.deepcopy(field))
            pointcloud_msg.is_bigendian=False
            pointcloud_msg.point_step=16
            pointcloud_msg.row_step=0
            pointcloud_msg.is_dense=True
            pointcloud_msg.data=[]

            #trajectory
            trajectory_msg=autoware_auto_planning_msgs.msg.Trajectory()
            trajectory_msg.header.frame_id=map_frame
            trajectory_msg.header.stamp=self.node.get_clock().now().to_msg()
            for x in range(10):
                trajectory_point=autoware_auto_planning_msgs.msg.TrajectoryPoint()
                trajectory_point.pose.position.x=x/10.0
                trajectory_point.pose.orientation.w=1.0
                trajectory_point.longitudinal_velocity_mps=25/3
                trajectory_msg.points.append(copy.deepcopy(trajectory_point))

            #odometry
            odom_msg=nav_msgs.msg.Odometry()
            odom_msg.header.frame_id=map_frame
            odom_msg.header.stamp=self.node.get_clock().now().to_msg()
            odom_msg.child_frame_id=base_link_frame
            odom_msg.pose.pose.orientation.w=1.0
            odom_msg.twist.twist.linear.x=0.0

            #object
            object_msg=autoware_auto_perception_msgs.msg.PredictedObjects()
            object_msg.header.frame_id=map_frame
            object_msg.header.stamp=self.node.get_clock().now().to_msg()
        
            #print("spin")
            object_pub.publish(object_msg)
            odom_pub.publish(odom_msg)
            pointcloud_pub.publish(pointcloud_msg)
            trajectory_pub.publish(trajectory_msg)

            rclpy.spin_once(self.node, timeout_sec=0.1)
        #self.assertEqual(10,9) 





"""
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        calc_result=5+5
        self.assertEqual(10,calc_result) 
        #launch_testing.asserts.assertExitCodes(proc_info)
        """