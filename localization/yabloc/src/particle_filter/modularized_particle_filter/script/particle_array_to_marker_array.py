#!/usr/bin/env python3

import copy
import numpy

import rclpy
from rclpy.node import Node

import modularized_particle_filter_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg


class ParticleArrayToMarkerArray(Node):
    def __init__(self):
        super().__init__('particle_array_to_marker_array')

        self._publisher = self.create_publisher(
            visualization_msgs.msg.MarkerArray, "pub/marker_array", 10)

        self._subscription = self.create_subscription(
            modularized_particle_filter_msgs.msg.ParticleArray, "sub/particle_array", self.on_particle_array, 10)

        self.__marker_array = None

    def on_particle_array(self, msg):
        if self.__marker_array is None or len(self.__marker_array.markers) != len(msg.particles):
            self.__marker_array = visualization_msgs.msg.MarkerArray()
            for i in range(len(msg.particles)):
                marker = visualization_msgs.msg.Marker()
                marker.header.frame_id = "map"
                marker.frame_locked = True
                marker.id = i
                marker.type = visualization_msgs.msg.Marker.ARROW
                marker.pose.position.x = 0.0  # p.x()
                marker.pose.position.y = 0.0  # p.y()
                marker.pose.position.z = 0.0  # p.z()
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale = geometry_msgs.msg.Vector3()
                marker.scale.x = 0.025
                marker.scale.y = 0.05
                marker.scale.z = 0.025
                marker.color.r = 1.0
                marker.color.g = 0.1
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.points.append(geometry_msgs.msg.Point())
                marker.points.append(geometry_msgs.msg.Point())
                marker.points[1].x = 0.1
                self.__marker_array.markers.append(marker)

        max_weight = max(map(lambda particle: particle.weight, msg.particles))

        for i, particle in enumerate(msg.particles):
            self.__marker_array.markers[i].pose = copy.deepcopy(particle.pose)
            if max_weight != 0:
                self.__marker_array.markers[i].color.r, \
                self.__marker_array.markers[i].color.g, \
                self.__marker_array.markers[i].color.b = ParticleArrayToMarkerArray.calculate_weight_color(
                    particle.weight / max_weight)
            else:
                self.__marker_array.markers[i].color.r = 0.2
                self.__marker_array.markers[i].color.g = 0.2
                self.__marker_array.markers[i].color.b = 0.2

        self._publisher.publish(self.__marker_array)

    @staticmethod
    def sigmoid(x, gain=1, offset_x=0):
        return ((numpy.tanh(((x+offset_x)*gain)/2)+1)/2)

    @staticmethod
    def calculate_weight_color(x, gain=10, offset_x=0.2, offset_green=0.6):
        x = (x * 2) - 1
        red = ParticleArrayToMarkerArray.sigmoid(x, gain, -1*offset_x)
        blue = 1-ParticleArrayToMarkerArray.sigmoid(x, gain, offset_x)
        green = ParticleArrayToMarkerArray.sigmoid(x, gain, offset_green) + \
            (1-ParticleArrayToMarkerArray.sigmoid(x, gain, -1*offset_green)) - 1.0
        return red, green, blue


def main(args=None):
    rclpy.init(args=args)

    node = ParticleArrayToMarkerArray()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
