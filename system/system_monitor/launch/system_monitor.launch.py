# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import os

import launch
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):

    cpu_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "cpu_monitor.param.yaml",
    )

    hdd_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "hdd_monitor.param.yaml",
    )
    mem_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "mem_monitor.param.yaml",
    )
    net_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "net_monitor.param.yaml",
    )
    ntp_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "ntp_monitor.param.yaml",
    )
    process_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "process_monitor.param.yaml",
    )
    gpu_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "gpu_monitor.param.yaml",
    )
    voltage_monitor_config_file = os.path.join(
        LaunchConfiguration("tier4_system_launch_param_path").perform(context),
        "system_monitor",
        "voltage_monitor.param.yaml",
    )

    with open(cpu_monitor_config_file, "r") as f:
        cpu_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    cpu_monitor = ComposableNode(
        package="system_monitor",
        plugin="CPUMonitor",
        name="cpu_monitor",
        parameters=[
            cpu_monitor_config,
        ],
    )
    with open(hdd_monitor_config_file, "r") as f:
        hdd_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    hdd_monitor = ComposableNode(
        package="system_monitor",
        plugin="HddMonitor",
        name="hdd_monitor",
        parameters=[
            hdd_monitor_config,
        ],
    )
    with open(mem_monitor_config_file, "r") as f:
        mem_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    mem_monitor = ComposableNode(
        package="system_monitor",
        plugin="MemMonitor",
        name="mem_monitor",
        parameters=[
            mem_monitor_config,
        ],
    )
    with open(net_monitor_config_file, "r") as f:
        net_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    net_monitor = ComposableNode(
        package="system_monitor",
        plugin="NetMonitor",
        name="net_monitor",
        parameters=[
            net_monitor_config,
        ],
    )
    with open(ntp_monitor_config_file, "r") as f:
        ntp_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    ntp_monitor = ComposableNode(
        package="system_monitor",
        plugin="NTPMonitor",
        name="ntp_monitor",
        parameters=[
            ntp_monitor_config,
        ],
    )
    with open(process_monitor_config_file, "r") as f:
        process_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    process_monitor = ComposableNode(
        package="system_monitor",
        plugin="ProcessMonitor",
        name="process_monitor",
        parameters=[
            process_monitor_config,
        ],
    )
    with open(gpu_monitor_config_file, "r") as f:
        gpu_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    gpu_monitor = ComposableNode(
        package="system_monitor",
        plugin="GPUMonitor",
        name="gpu_monitor",
        parameters=[
            gpu_monitor_config,
        ],
    )
    with open(voltage_monitor_config_file, "r") as f:
        voltage_monitor_config = yaml.safe_load(f)["/**"]["ros__parameters"]
    voltage_monitor = ComposableNode(
        package="system_monitor",
        plugin="VoltageMonitor",
        name="voltage_monitor",
        parameters=[
            voltage_monitor_config,
        ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="system_monitor_container",
        namespace="system_monitor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            cpu_monitor,
            hdd_monitor,
            mem_monitor,
            net_monitor,
            ntp_monitor,
            process_monitor,
            gpu_monitor,
            voltage_monitor,
        ],
        output="screen",
    )
    return [container]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
