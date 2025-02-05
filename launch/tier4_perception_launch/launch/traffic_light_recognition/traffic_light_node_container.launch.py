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

from itertools import chain
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def launch_setup(context, *args, **kwargs):

    # Load all camera namespaces
    all_camera_namespaces = LaunchConfiguration("all_camera_namespaces").perform(context)

    # Convert string to list
    all_camera_namespaces = yaml.load(all_camera_namespaces, Loader=yaml.FullLoader)
    if not isinstance(all_camera_namespaces, list):
        raise ValueError(
            "all_camera_namespaces is not a list. You should declare it like `['camera6', 'camera7']`."
        )
    if not all((isinstance(v, str) for v in all_camera_namespaces)):
        raise ValueError(
            "all_camera_namespaces is not a list of strings. You should declare it like `['camera6', 'camera7']`."
        )

    # Create containers for all cameras
    traffic_light_recognition_containers = [
        create_traffic_light_node_container(namespace, context, *args, **kwargs)
        for namespace in all_camera_namespaces
    ]
    traffic_light_recognition_containers = list(chain(*traffic_light_recognition_containers))

    return traffic_light_recognition_containers


def create_traffic_light_node_container(namespace, context, *args, **kwargs):
    camera_arguments = {
        "input/image": f"/sensing/camera/{namespace}/image_raw",
        "output/rois": f"/perception/traffic_light_recognition/{namespace}/detection/rois",
        "output/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/traffic_signals",
        "output/car/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/car/traffic_signals",
        "output/pedestrian/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/pedestrian/traffic_signals",
    }

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    fine_detector_model_param = ParameterFile(
        param_file=LaunchConfiguration("fine_detector_param_path").perform(context),
        allow_substs=True,
    )
    car_traffic_light_classifier_model_param = ParameterFile(
        param_file=LaunchConfiguration("car_classifier_param_path").perform(context),
        allow_substs=True,
    )
    pedestrian_traffic_light_classifier_model_param = ParameterFile(
        param_file=LaunchConfiguration("pedestrian_classifier_param_path").perform(context),
        allow_substs=True,
    )

    container = ComposableNodeContainer(
        name="traffic_light_node_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_traffic_light_classifier",
                plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
                name="car_traffic_light_classifier",
                namespace="classification",
                parameters=[car_traffic_light_classifier_model_param, {"build_only": False}],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/output/traffic_signals", "classified/car/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_classifier",
                plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
                name="pedestrian_traffic_light_classifier",
                namespace="classification",
                parameters=[pedestrian_traffic_light_classifier_model_param, {"build_only": False}],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/output/traffic_signals", "classified/pedestrian/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_visualization",
                plugin="autoware::traffic_light::TrafficLightRoiVisualizerNode",
                name="traffic_light_roi_visualizer",
                parameters=[create_parameter_dict("enable_fine_detection", "use_image_transport")],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/input/rough/rois", "detection/rough/rois"),
                    (
                        "~/input/traffic_signals",
                        camera_arguments["output/traffic_signals"],
                    ),
                    ("~/output/image", "debug/rois"),
                    ("~/output/image/compressed", "debug/rois/compressed"),
                    ("~/output/image/compressedDepth", "debug/rois/compressedDepth"),
                    ("~/output/image/theora", "debug/rois/theora"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        output="both",
    )

    decompressor_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_image_transport_decompressor",
                plugin="autoware::image_preprocessor::ImageTransportDecompressor",
                name="traffic_light_image_decompressor",
                namespace=namespace,
                parameters=[{"encoding": "rgb8"}],
                remappings=[
                    (
                        "~/input/compressed_image",
                        [camera_arguments["input/image"], "/compressed"],
                    ),
                    ("~/output/raw_image", camera_arguments["input/image"]),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_image_decompressor")),
    )

    fine_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_traffic_light_fine_detector",
                plugin="autoware::traffic_light::TrafficLightFineDetectorNode",
                name="traffic_light_fine_detector",
                namespace=f"{namespace}/detection",
                parameters=[fine_detector_model_param, {"build_only": False}],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", "rough/rois"),
                    ("~/expect/rois", "expect/rois"),
                    ("~/output/rois", camera_arguments["output/rois"]),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_fine_detection")),
    )

    return [
        GroupAction([PushRosNamespace(namespace), container]),
        decompressor_loader,
        fine_detector_loader,
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    fine_detector_share_dir = get_package_share_directory("autoware_traffic_light_fine_detector")
    classifier_share_dir = get_package_share_directory("autoware_traffic_light_classifier")
    add_launch_arg("all_camera_namespaces", "[camera6, camera7]")
    add_launch_arg("enable_image_decompressor", "True")
    add_launch_arg("enable_fine_detection", "True")
    add_launch_arg("use_image_transport", "True")

    # traffic_light_fine_detector
    add_launch_arg(
        "fine_detector_param_path",
        os.path.join(fine_detector_share_dir, "config", "traffic_light_fine_detector.param.yaml"),
    )

    # traffic_light_classifier
    add_launch_arg(
        "car_classifier_param_path",
        os.path.join(classifier_share_dir, "config", "car_traffic_light_classifier.param.yaml"),
    )
    add_launch_arg(
        "pedestrian_classifier_param_path",
        os.path.join(
            classifier_share_dir,
            "config",
            "pedestrian_traffic_light_classifier.param.yaml",
        ),
    )

    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")

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
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            OpaqueFunction(function=launch_setup),
        ]
    )
