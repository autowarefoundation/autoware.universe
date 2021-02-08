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
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import AnonName, LaunchConfiguration


def generate_launch_description():

    ns = 'euclidean_cluster'
    pkg = 'euclidean_cluster'

    # declare launch arguments
    input_pointcloud_param = DeclareLaunchArgument(
        'input_pointcloud',
        default_value='/sensing/lidar/no_ground/pointcloud')

    input_map_param = DeclareLaunchArgument(
        'input_map',
        default_value='/map/pointcloud_map')

    output_clusters_param = DeclareLaunchArgument(
        'output_clusters',
        default_value='clusters')

    use_pointcloud_map_param = DeclareLaunchArgument(
        'use_pointcloud_map',
        default_value='false')

    # set voxel grid filter as a component
    voxel_grid_filter_component = ComposableNode(
        package='voxel_grid_filter',
        plugin='pcl::VoxelGrid',
        name=AnonName('voxel_grid_filter'),
        remappings=[('input', LaunchConfiguration('input_pointcloud')),
                    ('output', 'voxel_grid_filtered/pointcloud')],
        parameters=[
            {
                'filter_field_name': 'z',
                'filter_limit_min': 0.1,
                'filter_limit_max': 2.5,
                'filter_limit_negative': False,
                'leaf_size': 0.1,
                'input_frame': 'base_link',
                'output_frame': 'base_link',
            }
        ]
    )

    # set compare map filter as a component
    compare_map_filter_component = ComposableNode(
        package='pointcloud_preprocessor',
        plugin='pointcloud_preprocessor::VoxelBasedCompareMapFilterComponent',
        name=AnonName('compare_map_filter'),
        remappings=[('input', 'voxel_grid_filtered/pointcloud'),
                    ('map', LaunchConfiguration('input_map')),
                    ('output', 'compare_map_filtered/pointcloud')]
    )

    use_map_euclidean_cluster_component = ComposableNode(
        package=pkg,
        plugin='euclidean_cluster::EuclideanClusterNodelet',
        name=AnonName('euclidean_cluster'),
        remappings=[('input', 'compare_map_filtered/pointcloud'),
                    ('output', LaunchConfiguration('output_clusters'))],
        parameters=[
            {
                'target_frame': 'base_link',
                'use_height': False,
                'tolerance': 0.7,
                'min_cluster_size': 10,
                'max_cluster_size': 1000
            }
        ]
    )

    disuse_map_euclidean_cluster_component = ComposableNode(
        package=pkg,
        plugin='euclidean_cluster::EuclideanClusterNodelet',
        name=AnonName('euclidean_cluster'),
        remappings=[('input', 'voxel_grid_filtered/pointcloud'),
                    ('output', LaunchConfiguration('output_clusters'))],
        parameters=[
            {
                'target_frame': 'base_link',
                'use_height': False,
                'tolerance': 0.7,
                'min_cluster_size': 10,
                'max_cluster_size': 1000
            }
        ]
    )

    container = ComposableNodeContainer(
        name='euclidean_cluster_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[voxel_grid_filter_component],
        output='screen',
    )

    use_map_loader = LoadComposableNodes(
        composable_node_descriptions=[compare_map_filter_component,
                                      use_map_euclidean_cluster_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration('use_pointcloud_map')),
    )

    disuse_map_loader = LoadComposableNodes(
        composable_node_descriptions=[disuse_map_euclidean_cluster_component],
        target_container=container,
        condition=UnlessCondition(LaunchConfiguration('use_pointcloud_map')),
    )

    return launch.LaunchDescription([
        input_pointcloud_param,
        input_map_param,
        output_clusters_param,
        use_pointcloud_map_param,
        container,
        use_map_loader,
        disuse_map_loader
    ])
