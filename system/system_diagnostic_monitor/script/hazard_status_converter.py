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

import enum

from autoware_adapi_v1_msgs.msg import OperationModeState
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.time
from tier4_system_msgs.msg import DiagnosticGraph
from tier4_system_msgs.msg import HazardStatus


class FaultLevel(enum.Enum):
    NF = 1
    SF = 2
    LF = 3
    SPF = 4


def get_mode_name(mode):
    if mode == OperationModeState.STOP:
        return "/autoware/modes/stop"
    elif mode == OperationModeState.AUTONOMOUS:
        return "/autoware/modes/autonomous"
    if mode == OperationModeState.LOCAL:
        return "/autoware/modes/local"
    if mode == OperationModeState.REMOTE:
        return "/autoware/modes/remote"


class DiagNode:
    def __init__(self, node):
        self.node = node
        self.view = []

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
            return FaultLevel.NF
        if self.view == DiagnosticStatus.OK:
            return FaultLevel.SF
        if self.level == DiagnosticStatus.WARN:
            return FaultLevel.LF
        if self.view == DiagnosticStatus.WARN:
            return FaultLevel.LF
        return FaultLevel.SPF


class HazardStatusConverter(rclpy.node.Node):
    def __init__(self):
        super().__init__("hazard_status_converter")
        self.create_interface()
        self.mode_name = None
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
        self.pub_hazard = self.create_publisher(HazardStatus, "/hazard_status", 1)

    def on_modes(self, msg):
        self.mode_name = get_mode_name(msg.mode)

    def on_graph(self, msg):
        nodes = [DiagNode(node) for node in msg.nodes]
        level = DiagnosticStatus.OK
        for node in nodes:
            if node.name.startswith("/autoware/modes/"):
                if node.name == self.mode_name:
                    level = node.level
                    node.view = [node.level]
                else:
                    node.view = [DiagnosticStatus.OK]

        for node in reversed(nodes):
            node.view = min(node.level, max(node.view, default=DiagnosticStatus.OK))
            for index in node.links:
                nodes[index].view.append(node.view)

        faults = {level: [] for level in FaultLevel}
        for node in reversed(nodes):
            faults[node.category].append(node.node.status)

        hazards = HazardStatus()
        hazards.level = int.from_bytes(level, "little")
        hazards.emergency = DiagnosticStatus.ERROR <= level
        hazards.diagnostics_nf = faults[FaultLevel.NF]
        hazards.diagnostics_sf = faults[FaultLevel.SF]
        hazards.diagnostics_lf = faults[FaultLevel.LF]
        hazards.diagnostics_spf = faults[FaultLevel.SPF]
        self.pub_hazard.publish(hazards)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(HazardStatusConverter())
    rclpy.shutdown()
