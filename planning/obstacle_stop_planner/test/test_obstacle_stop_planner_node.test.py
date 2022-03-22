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
import math
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
import struct
from ros2param.api import call_get_parameters

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
        params=call_get_parameters( node=self.node,
                                    node_name='/obstacle_stop_planner_node',
                                    parameter_names=[
                                        'stop_planner.stop_margin',
                                        'wheel_base',
                                        'front_overhang',
                                        'stop_planner.expand_stop_range',
                                        'wheel_tread',
                                        'left_overhang',
                                        'right_overhang'
                                        ])
        self.stop_margin=params.values[0].double_value;
        wheel_base=params.values[1].double_value;
        front_overhang=params.values[2].double_value;
        self.base_link_to_front=wheel_base+front_overhang
        expand_stop_range=params.values[3].double_value;
        wheel_tread=params.values[4].double_value;
        left_overhang=params.values[5].double_value;
        right_overhang=params.values[6].double_value;
        self.left_limit_line=wheel_tread/2.0+left_overhang+expand_stop_range
        self.right_limit_line=wheel_tread/2.0+right_overhang+expand_stop_range
        

    def tearDown(self):
        self.node.destroy_node()

    def init_messages(self):
        #message init
        #frame id
        map_frame="map"
        base_link_frame="base_link"
        x=0
        y=0
        #pointcloud2 
        self.pointcloud_msg=sensor_msgs.msg.PointCloud2()
        self.pointcloud_msg.header.frame_id=base_link_frame
        self.pointcloud_msg.header.stamp=self.node.get_clock().now().to_msg()
        self.pointcloud_msg.height=1
        self.pointcloud_msg.width=0
        field=sensor_msgs.msg.PointField()
        field.name="x"
        field.offset=0
        field.datatype=7
        field.count=1
        self.pointcloud_msg.fields.append(copy.deepcopy(field))
        field.name="y"
        field.offset=4
        self.pointcloud_msg.fields.append(copy.deepcopy(field))
        field.name="z"
        field.offset=8
        self.pointcloud_msg.fields.append(copy.deepcopy(field))
        self.pointcloud_msg.is_bigendian=False
        self.pointcloud_msg.point_step=16
        self.pointcloud_msg.row_step=self.pointcloud_msg.width*self.pointcloud_msg.point_step
        self.pointcloud_msg.is_dense=True

        #trajectory
        self.trajectory_resolution=0.1
        self.trajectory_length=100.0
        trajectory_element=int(self.trajectory_length/self.trajectory_resolution)
        self.trajectory_msg=autoware_auto_planning_msgs.msg.Trajectory()
        self.trajectory_msg.header.frame_id=map_frame
        self.trajectory_msg.header.stamp=self.node.get_clock().now().to_msg()
        for x in range(trajectory_element):
            trajectory_point=autoware_auto_planning_msgs.msg.TrajectoryPoint()
            trajectory_point.pose.position.x=x*self.trajectory_resolution
            trajectory_point.pose.orientation.w=1.0
            trajectory_point.longitudinal_velocity_mps=25/3
            self.trajectory_msg.points.append(copy.deepcopy(trajectory_point))

        #odometry
        self.odom_msg=nav_msgs.msg.Odometry()
        self.odom_msg.header.frame_id=map_frame
        self.odom_msg.header.stamp=self.node.get_clock().now().to_msg()
        self.odom_msg.child_frame_id=base_link_frame
        self.odom_msg.pose.pose.orientation.w=1.0
        self.odom_msg.twist.twist.linear.x=0.0

        #object
        self.object_msg=autoware_auto_perception_msgs.msg.PredictedObjects()
        self.object_msg.header.frame_id=map_frame
        self.object_msg.header.stamp=self.node.get_clock().now().to_msg()
    
    def set_pointcloud_messages(self,x,y):
        self.pointcloud_msg.width=1
        self.pointcloud_msg.row_step=self.pointcloud_msg.width*self.pointcloud_msg.point_step
        self.pointcloud_msg.data=[]
        for i in range(0,self.pointcloud_msg.width):
            self.pointcloud_msg.data=self.point_to_list(x,y,i/10,0.0)+self.pointcloud_msg.data 

    def point_to_list(self,x,y,z,i):
        point_list=struct.pack('<f',x)+struct.pack('<f',y)+struct.pack('<f',z)+struct.pack('<f',i)
        return point_list

    def clip(self,val,min_val,max_val):
        return max(min(val,max_val),min_val)

    def trajectory_evaluation(self,x,y,trajectory):
        if self.left_limit_line >= y and y >= -self.right_limit_line:
            stop_line=self.clip(x-(self.stop_margin+self.base_link_to_front),0,self.trajectory_length)
            stop_element=int(stop_line/self.trajectory_resolution)+1
            check_vel=(trajectory.points[stop_element].longitudinal_velocity_mps == 0.0)
            return check_vel
        else:
            check_vel=math.isclose(trajectory.points[-1].longitudinal_velocity_mps,self.trajectory_msg.points[-1].longitudinal_velocity_mps,abs_tol=0.01)
            return check_vel

    def trajectory_callback(self,msg):
        result_val=self.trajectory_evaluation(self.current_x,self.current_y,msg)
        #print("x:",self.current_x," , y:",self.current_y," , result:",result_val)
        self.callback_flag=True
        if result_val:
            self.true_count+=1

    def test_linear_trajectory(self):
        #publisher
        pointcloud_pub = self.node.create_publisher(sensor_msgs.msg.PointCloud2, 'input/pointcloud', 10)
        trajectory_pub = self.node.create_publisher(autoware_auto_planning_msgs.msg.Trajectory, 'input/trajectory', 10)
        odom_pub = self.node.create_publisher(nav_msgs.msg.Odometry, 'input/odometry', 10)
        object_pub = self.node.create_publisher(autoware_auto_perception_msgs.msg.PredictedObjects, 'input/objects', 10)

        #subscliber
        trajectory_sub = self.node.create_subscription(autoware_auto_planning_msgs.msg.Trajectory, 'output/trajectory', self.trajectory_callback, 10)

        self.init_messages()
        self.current_x=0.0
        self.current_y=0.0
        self.callback_flag=False
        self.true_count=0

        #while rclpy.ok():
        for ix in range(0,10):
            for iy in range(10):
                self.current_x=ix*10.0
                self.current_y=iy/2.0-2.5
                self.set_pointcloud_messages(self.current_x,self.current_y)
                
                #update time stamp
                self.object_msg.header.stamp=self.node.get_clock().now().to_msg()
                self.odom_msg.header.stamp=self.node.get_clock().now().to_msg()
                self.pointcloud_msg.header.stamp=self.node.get_clock().now().to_msg()
                self.trajectory_msg.header.stamp=self.node.get_clock().now().to_msg()
                
                object_pub.publish(self.object_msg)
                odom_pub.publish(self.odom_msg)
                pointcloud_pub.publish(self.pointcloud_msg)
                trajectory_pub.publish(self.trajectory_msg)

                self.callback_flag=False
                while not self.callback_flag:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(self.true_count,100) 





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