#!/usr/bin/env python3

import copy
import numpy

import rclpy
from rclpy.node import Node

import modularized_particle_filter_msgs.msg
import geometry_msgs.msg


class ParticleArrayToPoseArray(Node):
    def __init__(self):
        super().__init__('particle_array_to_pose_array')

        self._publisher = self.create_publisher(
            geometry_msgs.msg.PoseArray, "pub/pose_array", 10)

        self._subscription = self.create_subscription(
            modularized_particle_filter_msgs.msg.ParticleArray, "sub/particle_array", self.on_particle_array, 10)

        self.__pose_array = None

    def on_particle_array(self, msg):
        if self.__pose_array is None or len(self.__pose_array.poses) != len(msg.particles):
            self.__pose_array = geometry_msgs.msg.PoseArray()
            for i in range(len(msg.particles)):
                self.__pose_array.poses.append(geometry_msgs.msg.Pose())

        self.__pose_array.header = msg.header;
        for i, particle in enumerate(msg.particles):
            self.__pose_array.poses[i] = copy.deepcopy(particle.pose)

        self._publisher.publish(self.__pose_array)


def main(args=None):
    rclpy.init(args=args)

    node = ParticleArrayToPoseArray()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
