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

from graphviz import Digraph
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from tier4_system_msgs.msg import DiagnosticGraph


class MyNode(Node):
    def __init__(self):
        super().__init__("system_diagnostic_graph_debug")
        qos_struct = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_struct = self.create_subscription(
            DiagnosticGraph, "/diagnostics_graph_struct", self.on_struct, qos_struct
        )

    def on_struct(self, msg):
        print(msg)


def test():
    graph = Digraph()
    graph.attr("node", shape="circle")
    graph.edge("N0", "N1", "E1")
    graph.edge("N0", "N2", "E2")
    graph.view()


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(MyNode())
    rclpy.shutdown()
