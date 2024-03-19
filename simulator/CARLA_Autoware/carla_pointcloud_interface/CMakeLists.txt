cmake_minimum_required(VERSION 3.5)
project(carla_pointcloud_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)

include_directories(
  ${PCL_COMMON_INCLUDE_DIRS}
)

ament_auto_add_library(carla_pointcloud_interface SHARED
  src/carla_pointcloud_interface/carla_pointcloud_interface_node.cpp
)

rclcpp_components_register_node(carla_pointcloud_interface
  PLUGIN "PointCloudInterface"
  EXECUTABLE carla_pointcloud_interface_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
