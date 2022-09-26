# Copyright 2018 Autoware Foundation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import signal
from subprocess import DEVNULL
from subprocess import Popen

from autoware_auto_perception_msgs.msg import TrackedObjects
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class RunnerNode(Node):
    def __init__(self):
        super().__init__("autoware_workflow_runner_node")

        self.declare_parameter("launch_file", "")
        self.launch_file = self.get_parameter("launch_file").get_parameter_value().string_value

        self.declare_parameter("vehicle_model", "")
        self.vehicle_model = self.get_parameter("vehicle_model").get_parameter_value().string_value

        self.declare_parameter("sensor_model", "")
        self.sensor_model = self.get_parameter("sensor_model").get_parameter_value().string_value

        self.autoware_pid = None
        self.timer_subs_checker = None

        self.client_read_dataset_futures = []
        self.client_read_dataset = self.create_client(Trigger, "read_current_segment")
        while not self.client_read_dataset.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("service not available, waiting again...")

        self.client_read_frame_futures = []
        self.client_read_dataset_frame = self.create_client(Trigger, "send_frame")
        while not self.client_read_dataset_frame.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("service not available, waiting again...")

        self.sub_segment_finished = self.create_subscription(
            Bool, "segment_finished", self.segment_finished_callback, 1
        )

        self.sub_tracking = self.create_subscription(
            TrackedObjects,
            "/perception/object_recognition/tracking/objects",
            self.tracked_objects_callback,
            10,
        )

        self.read_dataset_request()

    def spin(self):

        while rclpy.ok():
            rclpy.spin_once(self)

            incomplete_read_dataset_futures = []
            for f in self.client_read_dataset_futures:
                if f.done():
                    response = f.result()
                    if response.success:
                        self.autoware_pid = self.run_autoware()
                        self.timer_subs_checker = self.create_timer(
                            2, self.wait_until_autoware_subs_ready
                        )
                else:
                    incomplete_read_dataset_futures.append(f)

            self.client_read_dataset_futures = incomplete_read_dataset_futures

            incomplete_send_frame_futures = []
            for f in self.client_read_frame_futures:
                if f.done():
                    response = f.result()
                else:
                    incomplete_send_frame_futures.append(f)

            self.client_read_frame_futures = incomplete_send_frame_futures

    def read_dataset_request(self):
        req = Trigger.Request()
        self.client_read_dataset_futures.append(self.client_read_dataset.call_async(req))

    def read_frame_request(self):
        req = Trigger.Request()
        self.client_read_frame_futures.append(self.client_read_dataset_frame.call_async(req))

    def segment_finished_callback(self, ready):
        self.get_logger().info("Autoware is being killed. ")
        self.kill_autoware(self.autoware_pid)
        self.read_dataset_request()

    def wait_until_autoware_subs_ready(self):

        self.get_logger().info("Waiting for Autoware's subscriber to be ready")

        if self.check_lidar_model_ready():
            self.get_logger().info("Autoware ready.")
            self.read_frame_request()
            self.destroy_timer(self.timer_subs_checker)

    def run_autoware(self):
        cmd = (
            "ros2 launch perception_benchmark_tool "
            + self.launch_file
            + " vehicle_model:="
            + self.vehicle_model
            + " sensor_model:="
            + self.sensor_model
        )
        launch_process = Popen(cmd, text=False, shell=True, stdout=DEVNULL)
        return launch_process.pid

    def kill_autoware(self, parent_pid, sig=signal.SIGTERM):
        try:
            parent = psutil.Process(parent_pid)
        except psutil.NoSuchProcess:
            return
        children = parent.children(recursive=True)
        for process in children:
            process.send_signal(sig)

    def check_lidar_model_ready(self):
        centerpoint_ready = self.count_publishers(
            "/perception/object_recognition/detection/centerpoint/objects"
        )
        apollo_ready = self.count_publishers(
            "/perception/object_recognition/detection/apollo/labeled_clusters"
        )
        return bool(centerpoint_ready or apollo_ready)

    def tracked_objects_callback(self, tracked_objects):
        self.read_frame_request()


def main(args=None):
    rclpy.init(args=args)
    autoware_workflow_runner_node = RunnerNode()
    autoware_workflow_runner_node.spin()
    rclpy.shutdown()
