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

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    ssd_fine_detector_share_dir = get_package_share_directory(
        'traffic_light_ssd_fine_detector'
    )
    classifier_share_dir = get_package_share_directory(
        'traffic_light_classifier'
    )
    add_launch_arg('enable_fine_detection', 'True')
    add_launch_arg('input/image', '/sensing/camera/traffic_light/image_raw')

    # traffic_light_ssd_fine_detector
    add_launch_arg('onnx_file',
                   os.path.join(ssd_fine_detector_share_dir, 'data', 'mb2-ssd-lite-tlr.onnx'))
    add_launch_arg('label_file',
                   os.path.join(ssd_fine_detector_share_dir, 'data', 'voc_labels_tl.txt'))
    add_launch_arg('fine_detector_precision', 'FP32')
    add_launch_arg('score_thresh', '0.7')
    add_launch_arg('max_batch_size', '8')
    add_launch_arg('approximate_sync', 'False')
    add_launch_arg('mean', '[0.5, 0.5, 0.5]')
    add_launch_arg('std', '[0.5, 0.5, 0.5]')

    # traffic_light_classifier
    add_launch_arg('classifier_type', '1')
    add_launch_arg('model_file_path',
                   os.path.join(classifier_share_dir,
                                'data',
                                'traffic_light_classifier_mobilenetv2.onnx'))
    add_launch_arg('label_file_path',
                   os.path.join(classifier_share_dir, 'data', 'lamp_labels.txt'))
    add_launch_arg('precision', 'fp16')
    add_launch_arg('input_c', '3')
    add_launch_arg('input_h', '224')
    add_launch_arg('input_w', '224')

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    container = ComposableNodeContainer(
        name='traffic_light_node_container',
        namespace='/perception/traffic_light_recognition',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='traffic_light_classifier',
                plugin='traffic_light::TrafficLightClassifierNodelet',
                name='traffic_light_classifier',
                parameters=[create_parameter_dict('approximate_sync', 'classifier_type',
                                                  'model_file_path', 'label_file_path',
                                                  'precision', 'input_c', 'input_h', 'input_w')],
                remappings=[('input/image', LaunchConfiguration('input/image')),
                            ('input/rois', 'rois'),
                            ('output/traffic_light_states', 'traffic_light_states')]
            ),
            ComposableNode(
                package='traffic_light_visualization',
                plugin='traffic_light::TrafficLightRoiVisualizerNodelet',
                name='traffic_light_roi_visualizer',
                parameters=[create_parameter_dict('enable_fine_detection')],
                remappings=[('input/image', LaunchConfiguration('input/image')),
                            ('input/rois', 'rois'),
                            ('input/rough/rois', 'rough/rois'),
                            ('output/image', 'debug/rois'),
                            ('output/image/compressed', 'debug/rois/compressed'),
                            ('output/image/compressedDepth', 'debug/rois/compressedDepth')]
            )
        ],
        output='both',
    )

    ssd_fine_detector_param = create_parameter_dict('onnx_file', 'label_file',
                                                    'score_thresh', 'max_batch_size',
                                                    'approximate_sync', 'mean', 'std')
    ssd_fine_detector_param['mode'] = LaunchConfiguration('fine_detector_precision')

    loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package='traffic_light_ssd_fine_detector',
                plugin='traffic_light::TrafficLightSSDFineDetectorNodelet',
                name='traffic_light_ssd_fine_detector',
                parameters=[ssd_fine_detector_param],
                remappings=[('input/image', LaunchConfiguration('input/image')),
                            ('input/rois', 'rough/rois'),
                            ('output/rois', 'rois')]
            ),
        ],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_fine_detection')),
    )

    return LaunchDescription(launch_arguments + [container, loader])
