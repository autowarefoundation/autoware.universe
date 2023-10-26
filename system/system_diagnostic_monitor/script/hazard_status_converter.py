#!/usr/bin/env python3

# Copyright 2023 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_adapi_v1_msgs.msg import OperationModeState
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.time
from tier4_system_msgs.msg import DiagnosticGraph
from tier4_system_msgs.msg import HazardStatus


def level_text(level):
    if level == DiagnosticStatus.OK:
        return "OK   "
    if level == DiagnosticStatus.WARN:
        return "WARN "
    if level == DiagnosticStatus.ERROR:
        return "ERROR"
    if level == DiagnosticStatus.STALE:
        return "STALE"
    if level is None:
        return "NONE "
    return "-----"


class DiagNode:
    def __init__(self, node):
        self.node = node
        self.view = DiagnosticStatus.OK

    @property
    def level(self):
        return self.node.status.level

    @property
    def name(self):
        return self.node.status.name

    @property
    def links(self):
        for link in self.node.links:
            if link.used:
                yield link.index

    @property
    def category(self):
        if self.level == DiagnosticStatus.OK:
            return "NF "
        if self.view == DiagnosticStatus.OK:
            return "SF "
        if self.view == DiagnosticStatus.WARN:
            return "LF "
        return "SPF"


class HazardStatusConverter(rclpy.node.Node):
    def __init__(self):
        super().__init__("hazard_status_converter")
        self.create_interface()
        self.mode_name = "/autoware/modes/autonomous"

    def create_interface(self):
        durable_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self.sub_graph = self.create_subscription(
            DiagnosticGraph, "/diagnostics_graph", self.on_graph, 1
        )
        self.sub_modes = self.create_subscription(
            OperationModeState, "/api/operation_mode/state", self.on_modes, durable_qos
        )
        self.pub_hazard = self.create_publisher(HazardStatus, "/hazard_status2", 1)

    def on_modes(self, msg):
        pass

    def on_graph(self, msg):
        nodes = [DiagNode(node) for node in msg.nodes]
        for node in nodes:
            for index in node.links:
                link = nodes[index]
                link.view = max(link.view, node.level)
            if node.name == self.mode_name:
                node.view = node.level

        print("=" * 100)
        for node in nodes:
            print(
                level_text(node.level), level_text(node.view), node.category, node.node.status.name
            )

        hazards = HazardStatus()
        self.pub_hazard.publish(hazards)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(HazardStatusConverter())
    rclpy.shutdown()
