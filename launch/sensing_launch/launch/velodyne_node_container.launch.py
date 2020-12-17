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

import launch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import uuid


def acceptable_unique_name(prefix):
    id = str(uuid.uuid4())
    # ros2 apparently doesn't accept the UUID with hyphens in node names
    return prefix + id.replace('-', '_')


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('model')
    add_launch_arg('launch_driver', 'True')
    add_launch_arg('calibration')
    add_launch_arg('device_ip', '192.168.1.201')
    add_launch_arg('sensor_frame', 'velodyne')
    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('container_name', 'velodyne_composable_node_container')
    add_launch_arg('min_range')
    add_launch_arg('max_range')
    add_launch_arg('pcap', '')
    add_launch_arg('port', '2368')
    add_launch_arg('read_fast', 'False')
    add_launch_arg('read_once', 'False')
    add_launch_arg('repeat_delay', '0.0')
    add_launch_arg('rpm', '600.0')
    add_launch_arg('laserscan_ring', '-1')
    add_launch_arg('laserscan_resolution', '0.007')
    add_launch_arg('num_points_thresholds', '300')
    add_launch_arg('invalid_intensity')
    add_launch_arg('frame_id', 'velodyne')
    add_launch_arg('gps_time', 'False')
    add_launch_arg('input_frame', LaunchConfiguration('base_frame'))
    add_launch_arg('output_frame', LaunchConfiguration('base_frame'))

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    # turn packets into pointcloud as in
    # https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_pointcloud/launch/velodyne_convert_node-VLP16-composed-launch.py
    nodes.append(ComposableNode(
        package='velodyne_pointcloud',
        plugin='velodyne_pointcloud::Convert',
        name='velodyne_convert_node',
        parameters=[create_parameter_dict('calibration', 'min_range', 'max_range',
                                          'num_points_thresholds', 'invalid_intensity', 'sensor_frame')],
        remappings=[('velodyne_points', 'pointcloud_raw'),
                    ('velodyne_points_ex', 'pointcloud_raw_ex')],
    )
    )

    cropbox_parameters = create_parameter_dict('input_frame', 'output_frame')
    cropbox_parameters['negative'] = True

    cropbox_remappings = [
        ('/min_x', '/vehicle_info/min_longitudinal_offset'),
        ('/max_x', '/vehicle_info/max_longitudinal_offset'),
        ('/min_z', '/vehicle_info/min_lateral_offset'),
        ('/max_z', '/vehicle_info/max_lateral_offset'),
        ('/min_z', '/vehicle_info/min_height_offset'),
        ('/max_z', '/vehicle_info/max_height_offset'),
    ]

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_self',
        remappings=[('/input', 'pointcloud_raw_ex'),
                    ('/output', 'self_cropped/pointcloud_ex')
                    ] + cropbox_remappings,
        parameters=[cropbox_parameters],
    )
    )

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_mirror',
        remappings=[('/input', 'self_cropped/pointcloud_ex'),
                    ('/output', 'mirror_cropped/pointcloud_ex'),
                    ] + cropbox_remappings,
        parameters=[cropbox_parameters],
    )
    )

    # TODO(fred-apex-ai) Still need the distortion component
    if False:
        nodes.append(ComposableNode(
            package='TODO',
            plugin='TODO',
            name='fix_distortion',
            remappings=[
                ('velodyne_points_ex', 'mirror_cropped/pointcloud_ex'),
                ('velodyne_points_interpolate', 'rectified/pointcloud'),
                ('velodyne_points_interpolate_ex', 'rectified/pointcloud_ex'),
            ],
        )
        )

    nodes.append(ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::RingOutlierFilterComponent',
        name='ring_outlier_filter',
        remappings=[
            ('/input', 'rectified/pointcloud_ex'),
            ('/output', 'outlier_filtered/pointcloud')
        ],
    )
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        # need unique name, otherwise all processes in same container and the node names then clash
        name=acceptable_unique_name('velodyne_node_container'),
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
    )

    driver_component = ComposableNode(
        package='velodyne_driver',
        plugin='velodyne_driver::VelodyneDriver',
        # node is created in a global context, need to avoid name clash
        name='velodyne_driver',
        parameters=[create_parameter_dict('device_ip', 'gps_time', 'read_once', 'read_fast',
                                          'repeat_delay', 'frame_id', 'model', 'rpm', 'port',
                                          'pcap')],
    )

    # one way to add a ComposableNode conditional on a launch argument to a
    # container. The `ComposableNode` itself doesn't accept a condition
    loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_driver')),
    )

    return launch.LaunchDescription(launch_arguments + [container, loader])
