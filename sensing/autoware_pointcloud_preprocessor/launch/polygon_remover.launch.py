# Copyright 2022 The Autoware Contributors
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

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def load_parameter_dic(package_share_directory, yaml_file_path):
    param_file = os.path.join(get_package_share_directory(package_share_directory), yaml_file_path)

    with open(param_file, "r") as file:
        param_dic = yaml.safe_load(file)["/**"]["ros__parameters"]

    return param_dic


def generate_launch_description():
    ns = "pointcloud_preprocessor"
    pkg = "pointcloud_preprocessor"

    polygon_remover_param = load_parameter_dic(
        "autoware_vehicle_info_utils", "config/polygon_remover.yaml"
    )

    param = load_parameter_dic("pointcloud_preprocessor", "config/filter_param_file.yaml")

    param.update(polygon_remover_param)

    my_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::PolygonRemoverComponent",
        name="polygon_remover",
        parameters=[param],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[my_component],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
        ]
    )
