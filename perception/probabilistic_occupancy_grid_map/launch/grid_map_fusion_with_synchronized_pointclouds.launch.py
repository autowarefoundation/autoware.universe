# Copyright 2021 Tier IV, Inc. All rights reserved.
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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml

# In this file, we use "ogm" as a meaning of occupancy grid map


# overwrite parameter
def overwrite_config(param_dict, launch_config_name, node_params_name, context):
    if LaunchConfiguration(launch_config_name).perform(context) != "":
        param_dict[node_params_name] = LaunchConfiguration(launch_config_name).perform(context)


# load parameter files
def load_config_file(context, configuration_name: str):
    config_file = LaunchConfiguration(configuration_name).perform(context)
    with open(config_file, "r") as f:
        config_param_dict = yaml.safe_load(f)["/**"]["ros__parameters"]
    return config_param_dict


# sanity check for the config file
def fusion_config_sanity_check(fusion_config: dict):
    listed_param_names = [
        "each_raw_pointcloud_topics",
        "each_ogm_output_topics",
        "each_ogm_sensor_frames",
        "each_ogm_reliabilities",
    ]
    param_length_list = []

    for param_name in listed_param_names:
        # parameters is not in the config file dict
        if param_name not in fusion_config:
            raise Exception("Error: " + param_name + " is not in the config file")
        # get len of the parameter
        param_length_list.append(len(fusion_config[param_name]))

    # check if the length of the parameters are the same
    if not all(x == param_length_list[0] for x in param_length_list):
        raise Exception("Error: the length of the parameters are not the same")


def launch_setup(context, *args, **kwargs):
    """Launch fusion based occupancy grid map creation nodes.

    1. describe occupancy grid map generation nodes for each sensor input
    2. describe occupancy grid map fusion node
    3. launch setting
    """
    # 1. launch occupancy grid map generation nodes for each sensor input

    # load fusion config parameter
    fusion_config = load_config_file(context, "fusion_config_file")
    fusion_config_sanity_check(fusion_config)

    # pointcloud based occupancy grid map nodes
    gridmap_generation_composable_nodes = []

    number_of_nodes = len(fusion_config["each_raw_pointcloud_topics"])

    for i in range(number_of_nodes):
        # load parameter file
        ogm_config = load_config_file(context, "ogm_config_file")
        ogm_config["scan_origin_frame"] = fusion_config["each_ogm_sensor_frames"][i]

        # generate composable node
        node = ComposableNode(
            package="probabilistic_occupancy_grid_map",
            plugin="occupancy_grid_map::PointcloudBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node_in_" + ogm_config["scan_origin_frame"],
            remappings=[
                ("~/input/obstacle_pointcloud", fusion_config["obstacle_pointcloud_topic"]),
                ("~/input/raw_pointcloud", fusion_config["each_raw_pointcloud_topics"][i]),
                ("~/output/occupancy_grid_map", fusion_config["each_ogm_output_topics"][i]),
            ],
            parameters=[ogm_config],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
        gridmap_generation_composable_nodes.append(node)

    # 2. launch occupancy grid map fusion node
    gridmap_fusion_node = [
        ComposableNode(
            package="probabilistic_occupancy_grid_map",
            plugin="grid_map_fusion::GridMapFusionNode",
            name="occupancy_grid_map_fusion_node",
            remappings=[
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[fusion_config],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    # 3. launch setting
    occupancy_grid_map_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=gridmap_generation_composable_nodes + gridmap_fusion_node,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=gridmap_generation_composable_nodes + gridmap_fusion_node,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return [occupancy_grid_map_container, load_composable_nodes]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "true"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("container_name", "occupancy_grid_map_container"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg(
                "ogm_config_file",
                get_package_share_directory("probabilistic_occupancy_grid_map")
                + "/config/pointcloud_based_occupancy_grid_map_for_fusion.param.yaml",
            ),
            add_launch_arg(
                "fusion_config_file",
                get_package_share_directory("probabilistic_occupancy_grid_map")
                + "/config/grid_map_fusion.param.yaml",
            ),
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
