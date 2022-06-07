import os
import sys

from enum import Enum
import copy
import pytest
import rclpy
from rclpy.executors import Executor
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import GearReport

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from clients.client_event_cmd import ClientEventCmdAsync

class GearMode(Enum):
    NONE = 0
    NEUTRAL = 1
    DRIVE = 2
    REVERSE = 20
    PARK = 22
    LOW = 23

node = None
publisher = None
sub_msg = None

def get_gear(msg):
    global node, publisher, sub_msg
    sub_msg = copy.deepcopy(msg)
    if msg.report == GearMode.DRIVE.value:
        #print(f'[Subscription] Gear Mode = {msg.report}')
        node.get_logger().info(f'Gear Mode : {msg.report}')
        msg = GearCommand()
        stamp = node.get_clock().now().to_msg()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = GearMode.PARK.value
        publisher.publish(msg)
    elif msg.report == GearMode.PARK.value:
        #print(f'[Subscription] Gear Mode = {msg.report}')
        node.get_logger().info(f'Gear Mode : {msg.report}')


def main(args=None):
    global node, publisher, sub_msg
    rclpy.init(args=args)

    event_cmd_client = ClientEventCmdAsync()
    event_cmd_client.send_request()

    node = rclpy.create_node("test_change_gearmode")
    publisher = node.create_publisher(GearCommand, 'autoware_auto_vehicle_msgs/GearCommand', 10)
    subscriber = node.create_subscription(GearReport, 'autoware_auto_vehicle_msgs/GearReport', get_gear, 10)
    subscriber

    while rclpy.ok():
        rclpy.spin_once(event_cmd_client)
        if event_cmd_client.future.done():
            result_msg = event_cmd_client.future.result()
            event_cmd_client.get_logger().info(f'Change Control Mode : {result_msg.response.gear}')
            break
    
    event_cmd_client.destroy_node()

    while rclpy.ok():
        rclpy.spin_once(node)
        if sub_msg != None:
            if sub_msg.report == GearMode.PARK.value:
                node.destroy_node()
                break
    
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
