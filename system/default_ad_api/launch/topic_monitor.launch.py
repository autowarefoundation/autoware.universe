# Copyright 2022 TIER IV, Inc.
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

import csv

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def create_node(row):
    package = FindPackageShare("topic_state_monitor")
    include = PathJoinSubstitution([package, "launch/topic_state_monitor.launch.xml"])
    arguments = [
        ("diag_name", "default_ad_api/topic_monitor/" + row["module"]),
        ("node_name_suffix", row["suffix"]),
        ("topic", row["topic"]),
        ("topic_type", row["topic_type"]),
        ("timeout", row["timeout"]),
        ("warn_rate", row["warn_rate"]),
        ("error_rate", row["error_rate"]),
        ("best_effort", row["best_effort"]),
        ("transient_local", row["transient_local"]),
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def launch_setup(context, *args, **kwargs):
    with open(LaunchConfiguration("config_file").perform(context)) as fp:
        nodes = [create_node(row) for row in csv.DictReader(fp)]
        return [node for node in nodes if node]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("config_file"),
            OpaqueFunction(function=launch_setup),
        ]
    )
