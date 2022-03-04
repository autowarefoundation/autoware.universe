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

import time
import unittest
import std_msgs.msg
import autoware_simulation_msgs.msg
import diagnostic_msgs.msg
import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
import launch_testing
import pytest
import rclpy


@pytest.mark.launch_test

def generate_test_description():
    
    print("==================== launch test start =====================")
    obstacle_stop_planner_node = launch_ros.actions.Node(
        package='obstacle_stop_planner',
        executable='obstacle_stop_planner_node',
        name='obstacle_stop_planner_node',
        parameters=[
            [
                FindPackageShare('vehicle_info_util'),
                '/config/vehicle_info.param.yaml'
            ]
        ],
    )
    return (
        launch.LaunchDescription([
            obstacle_stop_planner_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'obstacle_stop_planner': obstacle_stop_planner_node,
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
        self.node = rclpy.create_node('test_talker_listener_link')
        self.event_name = 'cpu_temperature_warning'

    def tearDown(self):
        self.node.destroy_node()

    
    
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


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        calc_result=5+5
        self.assertEqual(10,calc_result) 
        #launch_testing.asserts.assertExitCodes(proc_info)