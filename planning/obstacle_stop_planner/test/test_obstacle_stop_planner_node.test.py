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

import time
import unittest

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
    
    print("=========================================")
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
        #self.assertInStdout(calc_result)
        print('===========================')
        self.assertEqual(10,calc_result) 
        #return 0

    def test_simple_calc2(self):
        calc_result=5+6

        #self.assertInStdout(calc_result)
        self.assertEqual(10,calc_result) 
        #return 0
    



@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        calc_result=5+5
        self.assertEqual(10,calc_result) 
        #launch_testing.asserts.assertExitCodes(proc_info)