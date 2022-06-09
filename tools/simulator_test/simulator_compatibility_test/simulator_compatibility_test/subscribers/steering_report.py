import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from autoware_auto_vehicle_msgs.msg import SteeringReport

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.normpath(os.path.join(current_path, './')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))


class SubscriberSteeringReport(Node):
    
    def __init__(self):
        super().__init__('steering_report_subscriber')
        
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = qos_depth,
            durability = QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription_ = self.create_subscription(SteeringReport,
                                                      '/vehicle/status/steering_status',
                                                      self.get_steering,
                                                      QOS_RKL10V)
        
        self.received = []
        
    def get_steering(self, msg):
        self.received.append(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = SubscriberSteeringReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
