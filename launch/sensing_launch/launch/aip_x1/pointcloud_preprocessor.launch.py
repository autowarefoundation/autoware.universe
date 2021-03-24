
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(context):
    path = LaunchConfiguration('vehicle_param_file').perform(context)
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    p['vehicle_length'] = p['front_overhang'] + p['wheel_base'] + p['rear_overhang']
    p['vehicle_width'] = p['wheel_tread'] + p['left_overhang'] + p['right_overhang']
    p['min_longitudinal_offset'] = -p['rear_overhang']
    p['max_longitudinal_offset'] = p['front_overhang'] + p['wheel_base']
    p['min_lateral_offset'] = -(p['wheel_tread'] / 2.0 + p['right_overhang'])
    p['max_lateral_offset'] = p['wheel_tread'] / 2.0 + p['left_overhang']
    p['min_height_offset'] = 0.0
    p['max_height_offset'] = p['vehicle_height']
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration('vehicle_mirror_param_file').perform(context)
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return p


def launch_setup(context, *args, **kwargs):

    pkg = 'pointcloud_preprocessor'

    vehicle_info = get_vehicle_info(context)

    # set concat filter as a component
    concat_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
        name='concatenate_data',
        remappings=[('/output', 'concatenated/pointcloud')],
        parameters=[{
            'input_topics': ['/sensing/lidar/top/outlier_filtered/pointcloud',
                             '/sensing/lidar/front_left/mirror_cropped/pointcloud',
                             '/sensing/lidar/front_right/mirror_cropped/pointcloud',
                             '/sensing/lidar/front_center/mirror_cropped/pointcloud'],
            'output_frame': LaunchConfiguration('base_frame'),
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    # set PointCloud PassThrough Filter as a component
    passthrough_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PassThroughFilterComponent',
        name='passthrough_filter',
        remappings=[
            ('input', 'top/outlier_filtered/pointcloud'),
            ('output', 'concatenated/pointcloud'),
        ],
        parameters=[{
            'output_frame': LaunchConfiguration('base_frame'),
            'min_z': vehicle_info['min_height_offset'],
            'max_z': vehicle_info['max_height_offset'],
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter',
        remappings=[
            ('input', 'concatenated/pointcloud'),
            ('output', 'measurement_range_cropped/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': -50.0,
            'max_x': 100.0,
            'min_y': -50.0,
            'max_y': 50.0,
            'min_z': -0.5,
            'max_z': vehicle_info['max_height_offset'],
            'negative': False,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    ray_ground_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::RayGroundFilterComponent',
        name='ray_ground_filter',
        remappings=[
            ('input', 'measurement_range_cropped/pointcloud'),
            ('output', 'rough/no_ground/pointcloud'),
        ],
        parameters=[{
            'initial_max_slope': 10.0,
            'general_max_slope': 10.0,
            'local_max_slope': 10.0,
            'min_height_threshold': 0.3,
            'radial_divider_angle': 1.0,
            'concentric_divider_distance': 0.0,
            'use_vehicle_footprint': True,
            'min_x': vehicle_info['min_longitudinal_offset'],
            'max_x': vehicle_info['max_longitudinal_offset'],
            'min_y': vehicle_info['min_lateral_offset'],
            'max_y': vehicle_info['max_lateral_offset'],
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    short_height_obstacle_detection_area_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='short_height_obstacle_detection_area_filter',
        remappings=[
            ('input', 'front_center/mirror_cropped/pointcloud'),
            ('output', 'short_height_obstacle_detection_area/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': 0.0,
            'max_x': 15.6,  # max_x: 14.0m + base_link2livox_front_center distance 1.6m
            'min_y': -4.0,
            'max_y': 4.0,
            'min_z': -0.5,
            'max_z': 0.5,
            'negative': False,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    vector_map_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::Lanelet2MapFilterComponent',
        name='vector_map_filter',
        remappings=[
            ('input/pointcloud', 'short_height_obstacle_detection_area/pointcloud'),
            ('input/vector_map', '/map/vector_map'),
            ('output', 'vector_map_filtered/pointcloud'),
        ],
        parameters=[{
            'voxel_size_x': 0.25,
            'voxel_size_y': 0.25,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    ransac_ground_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::RANSACGroundFilterComponent',
        name='ransac_ground_filter',
        remappings=[
            ('input', 'vector_map_filtered/pointcloud'),
            ('output', 'short_height/no_ground/pointcloud'),
        ],
        parameters=[{
            'outlier_threshold': 0.1,
            'min_points': 400,
            'min_inliers': 200,
            'max_iterations': 50,
            'height_threshold': 0.12,
            'plane_slope_threshold': 10.0,
            'voxel_size_x': 0.2,
            'voxel_size_y': 0.2,
            'voxel_size_z': 0.2,
            'debug': False,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    concat_no_ground_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent',
        name='concatenate_no_ground_data',
        remappings=[('output', 'no_ground/concatenated/pointcloud')],
        parameters=[{
            'input_topics': ['/sensing/lidar/rough/no_ground/pointcloud',
                             '/sensing/lidar/short_height/no_ground/pointcloud'],
            'output_frame': LaunchConfiguration('base_frame'),
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    voxel_grid_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
        name='voxel_grid_filter',
        remappings=[
            ('input', 'no_ground/concatenated/pointcloud'),
            ('output', 'voxel_grid_filtered/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('base_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'voxel_size_x': 0.04,
            'voxel_size_y': 0.04,
            'voxel_size_z': 0.08,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    radius_search_2d_outlier_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::RadiusSearch2dOutlierFilterComponent',
        name='radius_search_2d_outlier_filter',
        remappings=[
            ('input', 'voxel_grid_filtered/pointcloud'),
            ('output', 'no_ground/pointcloud'),
        ],
        parameters=[{
            'search_radius': 0.2,
            'min_neighbors': 5,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    voxel_grid_outlier_filter_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::VoxelGridOutlierFilterComponent',
        name='voxel_grid_outlier_filter',
        remappings=[
            ('input', 'voxel_grid_filtered/pointcloud'),
            ('output', 'no_ground/pointcloud'),
        ],
        parameters=[{
            'voxel_size_x': 0.4,
            'voxel_size_y': 0.4,
            'voxel_size_z': 100.0,
            'voxel_points_threshold': 5,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }]
    )

    relay_component = ComposableNode(
        package='topic_tools',
        plugin='topic_tools::RelayNode',
        name='relay',
        parameters=[{
            'input_topic': '/sensing/lidar/top/outlier_filtered/pointcloud',
            'output_topic': '/sensing/lidar/pointcloud',
            'type': 'sensor_msgs/msg/PointCloud2',
            'history': 'keep_last',
            'depth': 5,
            'reliability': 'best_effort',
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            cropbox_component,
            ray_ground_filter_component,
            short_height_obstacle_detection_area_filter_component,
            vector_map_filter_component,
            ransac_ground_filter_component,
            concat_no_ground_component,
            voxel_grid_filter_component,
            relay_component,
        ],
        output='screen',
        parameters=[{
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False')
        }],
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_concat_filter')),
    )

    passthrough_loader = LoadComposableNodes(
        composable_node_descriptions=[passthrough_component],
        target_container=container,
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_concat_filter')),
    )

    # load concat or passthrough filter
    radius_search_2d_outlier_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[radius_search_2d_outlier_filter_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_radius_search')),
    )

    voxel_grid_outlier_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[voxel_grid_outlier_filter_component],
        target_container=container,
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_radius_search')),
    )
    return [container, concat_loader, passthrough_loader,
            radius_search_2d_outlier_filter_loader,
            voxel_grid_outlier_filter_loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('use_concat_filter', 'true')
    add_launch_arg('use_radius_search', 'true')
    add_launch_arg('vehicle_param_file')

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
