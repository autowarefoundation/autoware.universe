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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

import yaml


def get_vehicle_info(context):
    path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
    p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
    p["min_longitudinal_offset"] = -p["rear_overhang"]
    p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
    p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
    p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = p["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    top_components = []

    self_cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    self_cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    self_cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    self_cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    self_cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    self_cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    self_cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    self_cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    top_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace="/sensing/lidar/top",
            remappings=[
                ("input", "/lgsvl/top/pointcloud"),
                ("output", "/lgsvl/top/self_cropped/pointcloud"),
            ],
            parameters=[self_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    mirror_cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    mirror_cropbox_parameters["negative"] = True

    mirror_info = get_vehicle_mirror_info(context)
    # TODO hardcoded change to not detect the mirrors as obstacle in SVL
    # this seems caused by some delay in the pointcloud update which cause points of the mirror to "stay in place" for a short time while the car moves.
    # when the car moves the points stay behind their actual position and can get out of the crop box if it is too small.
    mirror_cropbox_parameters["min_x"] = 1.0 # mirror_info["min_longitudinal_offset"]
    mirror_cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    mirror_cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    mirror_cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    mirror_cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    mirror_cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    top_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace="/sensing/lidar/top",
            remappings=[
                ("input", "/lgsvl/top/self_cropped/pointcloud"),
                ("output", "/sensing/lidar/top/outlier_filtered/pointcloud"),
            ],
            parameters=[mirror_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    top_components.append(
        ComposableNode(
            package="topic_tools",
            plugin="topic_tools::RelayNode",
            name="pointcloud_relay",
            namespace="/sensing/lidar/top",
            parameters=[
                {
                    "input_topic": "/sensing/lidar/top/outlier_filtered/pointcloud",
                    "output_topic": "/sensing/lidar/top/rectified/pointcloud",
                    "type": "sensor_msgs/msg/PointCloud2",
                    "reliability": "best_effort",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    top_container = ComposableNodeContainer(
        name="velodyne_node_container",
        namespace="/sensing/lidar/top/pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=top_components,
        output="screen",
    )

    left_components=[]

    left_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace="/sensing/lidar/left",
            remappings=[
                ("input", "/lgsvl/left/pointcloud"),
                ("output", "/lgsvl/left/self_cropped/pointcloud"),
            ],
            parameters=[self_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    left_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace="/sensing/lidar/left",
            remappings=[
                ("input", "/lgsvl/left/self_cropped/pointcloud"),
                ("output", "/sensing/lidar/left/outlier_filtered/pointcloud"),
            ],
            parameters=[mirror_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    left_container = ComposableNodeContainer(
        name="velodyne_node_container",
        namespace="/sensing/lidar/left/pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=left_components,
        output="screen",
    )

    right_components=[]

    right_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace="/sensing/lidar/right",
            remappings=[
                ("input", "/lgsvl/right/pointcloud"),
                ("output", "/lgsvl/right/self_cropped/pointcloud"),
            ],
            parameters=[self_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    right_components.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace="/sensing/lidar/right",
            remappings=[
                ("input", "/lgsvl/right/self_cropped/pointcloud"),
                ("output", "/sensing/lidar/right/outlier_filtered/pointcloud"),
            ],
            parameters=[mirror_cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    right_container = ComposableNodeContainer(
        name="velodyne_node_container",
        namespace="/sensing/lidar/right/pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=right_components,
        output="screen",
    )

    return [
        top_container,
        left_container,
        right_container,
    ]



def generate_launch_description():

    return launch.LaunchDescription([
        DeclareLaunchArgument("vehicle_model"),
        DeclareLaunchArgument(
            "vehicle_param_file",
            default_value=[
                FindPackageShare(
                    [LaunchConfiguration("vehicle_model"), "_description"]
                ),
                "/config/vehicle_info.param.yaml",
            ],
        ),
        DeclareLaunchArgument(
            "vehicle_mirror_param_file",
            default_value=[
                FindPackageShare(
                    [LaunchConfiguration("vehicle_model"), "_description"]
                ),
                "/config/mirror.param.yaml",
            ],
        ),
        DeclareLaunchArgument(
            "input_frame", default_value="base_link"
        ),
        DeclareLaunchArgument(
            "output_frame", default_value="base_link"
        ),
        SetParameter(
            name="use_sim_time",
            value=True,
        ),
        OpaqueFunction(function=launch_setup),
        GroupAction([
            PushRosNamespace("/sensing/lidar"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("aip_xx1_launch"), "/launch/pointcloud_preprocessor.launch.py"]
                ),
                launch_arguments={
                    "base_frame": "base_link",
                    "use_concat_filter": "true",
                    "vehicle_param_file": LaunchConfiguration("vehicle_param_file"),
                    "use_intra_process": "true",
                    "use_multithread": "true",
                }.items(),
            )
        ]),
     ])