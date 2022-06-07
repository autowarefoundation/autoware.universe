import os
import sys

from enum import Enum
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

from client_event_cmd import ClientEventCmdAsync

class GearMode(Enum):
    NONE = 0
    NEUTRAL = 1
    DRIVE = 2
    REVERSE = 20
    PARK = 22
    LOW = 23

class TestGear(Node):
    def __init__(self):
        super().__init__("TestGear")
        self.publisher_ = self.create_publisher(GearCommand, 'autoware_auto_vehicle_msgs/GearCommand', 10)
        self.subscription_ = self.create_subscription(GearReport, 'autoware_auto_vehicle_msgs/GearReport', self.get_gear, 10)
        #self.pub_timer_ = self.create_timer(2, self.set_gear)
        self.idx = 0
        self.mode = [GearMode.DRIVE.value, GearMode.PARK.value]
        self.current_mode = self.mode[0]
    
    def set_gear(self):
        msg = GearCommand()
        stamp = self.get_clock().now().to_msg()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = self.mode[1]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publish : Gear Mode = {self.mode[self.idx % 2]}')
        self.idx += 1
        self.current_mode = self.mode[self.idx % 2]
    
    def get_gear(self, msg):
        if msg.report == GearMode.DRIVE.value:
            print(f'[Subscription] Gear Mode = {msg.report}')
            self.set_gear()
        elif msg.report == GearMode.PARK.value:
            print(f'[Subscription] Gear Mode = {msg.report}')
            self.publisher_.destroy()
            self.subscription_.destroy()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    event_cmd_client = ClientEventCmdAsync()
    event_cmd_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(event_cmd_client)
        if event_cmd_client.future.done():
            result_msg = event_cmd_client.future.result()
            event_cmd_client.get_logger().info(f'Change Control Mode : {result_msg.response.gear}')
            break
    
    event_cmd_client.destroy_node()
    test = TestGear()
    rclpy.spin(test)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
