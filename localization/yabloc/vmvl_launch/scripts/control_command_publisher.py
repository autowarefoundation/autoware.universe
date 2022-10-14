#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from std_msgs.msg import String
from std_msgs.msg import Float32
import numpy as np

class control_command_publisher(Node):
    def __init__(self):
        super().__init__('control_command_publisher')

        self.sub_command_ = self.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd',  self.command_callback, 10)

        self.pub_string_ = self.create_publisher(String, '/string/command', 10)
        self.pub_float_ = self.create_publisher(Float32, '/float/steer', 10)

    def command_callback(self, msg: AckermannControlCommand):
        
        commands={}
        commands['steer_angle(deg)']=msg.lateral.steering_tire_angle * 180/np.pi
        commands['steer_rate(deg/s)']=msg.lateral.steering_tire_rotation_rate * 180/np.pi
        commands['speed']=msg.longitudinal.speed
        commands['accel']=msg.longitudinal.acceleration
        commands['jerk ']=msg.longitudinal.jerk

        str_msg=String()
        str_msg.data='-- Control Command Status --\n'
        for key,value in commands.items():
            str_msg.data+= key+': '+'{:.3f}'.format(value)+'\n'

        self.pub_string_.publish(str_msg)
        
        float_msg=Float32()
        float_msg.data=msg.lateral.steering_tire_angle * 180/np.pi
        self.pub_float_.publish(float_msg)

def main(args=None):
    rclpy.init(args=args)

    converter = control_command_publisher()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
