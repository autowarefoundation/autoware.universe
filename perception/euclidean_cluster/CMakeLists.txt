cmake_minimum_required(VERSION 3.5)
project(euclidean_cluster)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common search filters)
ament_auto_find_build_dependencies()


include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(euclidean_cluster_nodelet SHARED
  src/euclidean_cluster_nodelet.cpp
)
target_link_libraries(euclidean_cluster_nodelet
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(euclidean_cluster_nodelet
  PLUGIN "euclidean_cluster::EuclideanClusterNodelet"
  EXECUTABLE euclidean_cluster_node
)

ament_auto_add_library(voxel_grid_based_euclidean_cluster_nodelet SHARED
  src/voxel_grid_based_euclidean_cluster_nodelet.cpp
)
target_link_libraries(voxel_grid_based_euclidean_cluster_nodelet
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(voxel_grid_based_euclidean_cluster_nodelet
  PLUGIN "euclidean_cluster::VoxelGridBasedEuclideanClusterNodelet"
  EXECUTABLE voxel_grid_based_euclidean_cluster_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
)
