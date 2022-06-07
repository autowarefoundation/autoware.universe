import os
import sys
import unittest
import pytest

import launch
import launch_ros
import launch_testing
import launch_testing_ros

import rclpy

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import GearReport

from grpc_ros2_bridge_test_automation.clients.client_event_cmd import ClientEventCmdAsync
from grpc_ros2_bridge_test_automation.publishers.gear_command import PublisherGearCommand, GearMode
from grpc_ros2_bridge_test_automation.subscribers.gear_report import SubscriberGearReport

"""
usage :
    1. colcon build --packages-select grpc_ros2_bridge_test_automation
    2. execute below on your terminal
launch_test src/grpc_ros2_bridge_test_automation/test/test_change_gear_mode.py
"""
@pytest.mark.launch_test
def generate_test_description():
    path_to_test = os.path.abspath(os.path.dirname(__file__))
    parent_path = os.path.abspath(os.path.join(path_to_test, '../'+'grpc_ros2_bridge_test_automation/'))
    clients_dir = os.path.abspath(os.path.join(parent_path, './clients/'))
    publishers_dir = os.path.abspath(os.path.join(parent_path, './publishers/'))
    subscribers_dir = os.path.abspath(os.path.join(parent_path, './subscribers/'))


    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    gear_command = launch.actions.ExecuteProcess(
        cmd=[sys.executable, publishers_dir+'/gear_command.py'],
        env=proc_env, output='log'
    )
    gear_report = launch.actions.ExecuteProcess(
        cmd=[sys.executable, subscribers_dir+'/gear_report.py'],
        env=proc_env, output='log'
    )
    #gear_command_node = launch_ros.actions.Node(
    #    executable=sys.executable,
    #    output = 'log',
    #    arguments=[os.path.join(publishers_dir, 'gear_command.py')],
    #)

    #gear_report_node = launch_ros.actions.Node(
    #    executable=sys.executable,
    #    output = 'log',
    #    arguments=[os.path.join(subscribers_dir, 'gear_report.py')]
    #)

    return (launch.LaunchDescription([
        gear_command,
        gear_report,
        #gear_command_node,
        #gear_report_node,
        launch_testing.actions.ReadyToTest(),]),
        { 
            'gear_command': gear_command,
            'gear_report' : gear_report
            #'gear_command': gear_command_node,
            #'gear_report' : gear_report_node
        }
    )


class TestGearCommandReport(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()
    
    def setUp(self) -> None:
        self.node = rclpy.create_node('test_gear_command_report')
    
    def tearDown(self) -> None:
        self.node.destroy_node()
    
    def test_1_set_morai_auto_mode(self, launch_service, gear_command, gear_report, proc_output):
        event_cmd_client = ClientEventCmdAsync()
        event_cmd_client.send_request()

        while rclpy.ok():
            rclpy.spin_once(event_cmd_client)
            if event_cmd_client.future.done():
                result_msg = event_cmd_client.future.result()
                self.assertEqual(result_msg.response.ctrl_mode, 3)
            break
    
    def test_2_set_gear_mode(self, launch_service, gear_command, gear_report, proc_output):
        msgs_rx = []

        sub = self.node.create_subscription(
            GearCommand,
            'autoware_auto_vehicle_msgs/GearCommand',
            lambda msg : msgs_rx.append(msg),
            10
        )

        try:
            while rclpy.ok():
                rclpy.spin_once(self.node)
                if len(msgs_rx) > 2:
                    break
            received = msgs_rx.pop()
            self.assertEqual(received.command, GearMode.PARK.value)
        finally:
            self.node.destroy_subscription(sub)
    
    def test_3_get_gear_mode(self, launch_service, gear_command, gear_report, proc_output):
        msgs_rx = []

        sub = self.node.create_subscription(
            GearReport,
            'autoware_auto_vehicle_msgs/GearReport',
            lambda msg : msgs_rx.append(msg),
            10
        )

        try:
            while rclpy.ok():
                rclpy.spin_once(self.node)
                if len(msgs_rx) > 2:
                    break
            received = msgs_rx.pop()
            self.assertEqual(received.report, GearMode.PARK.value)
        finally:
            self.node.destroy_subscription(sub)


if __name__ == '__main__':
    unittest.main()

