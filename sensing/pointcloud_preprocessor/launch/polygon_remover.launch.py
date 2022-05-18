import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    ns = "pointcloud_preprocessor"
    pkg = "pointcloud_preprocessor"

    param_file = os.path.join(
        get_package_share_directory("vehicle_info_util"), "config/polygon_remover.yaml"
    )

    with open(param_file, "r") as f:
        polygon_remover_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    my_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::PolygonRemoverComponent",
        name="polygon_remover",
        parameters=[
            {
                "polygon_vertices": polygon_remover_param["polygon_vertices"],
                "will_visualize": polygon_remover_param["will_visualize"],
            }
        ],
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
