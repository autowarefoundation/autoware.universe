cmake_minimum_required(VERSION 3.14)
project(probabilistic_occupancy_grid_map)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

# PointcloudBasedOccupancyGridMap
ament_auto_add_library(pointcloud_based_occupancy_grid_map SHARED
  src/pointcloud_based_occupancy_grid_map/pointcloud_based_occupancy_grid_map_node.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
  src/utils/utils.cpp
)

target_link_libraries(pointcloud_based_occupancy_grid_map
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(pointcloud_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::PointcloudBasedOccupancyGridMapNode"
  EXECUTABLE pointcloud_based_occupancy_grid_map_node
)

# LaserscanBasedOccupancyGridMap
ament_auto_add_library(laserscan_based_occupancy_grid_map SHARED
  src/laserscan_based_occupancy_grid_map/laserscan_based_occupancy_grid_map_node.cpp
  src/laserscan_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
  src/utils/utils.cpp
)

target_link_libraries(laserscan_based_occupancy_grid_map
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(laserscan_based_occupancy_grid_map
  PLUGIN "occupancy_grid_map::LaserscanBasedOccupancyGridMapNode"
  EXECUTABLE laserscan_based_occupancy_grid_map_node
)

# GridMapFusionNode
ament_auto_add_library(grid_map_fusion SHARED
  src/fusion/grid_map_fusion_node.cpp
  src/pointcloud_based_occupancy_grid_map/occupancy_grid_map.cpp
  src/updater/occupancy_grid_map_binary_bayes_filter_updater.cpp
  src/utils/utils.cpp
)

target_link_libraries(grid_map_fusion
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(grid_map_fusion
  PLUGIN "grid_map_fusion::GridMapFusionNode"
  EXECUTABLE grid_map_fusion_node
)

# Tests
if(BUILD_TESTING)
  ament_add_gtest(costmap_unit_tests
    test/cost_value_test.cpp)
  ament_add_gtest(fusion_policy_unit_tests
    test/fusion_policy_test.cpp)
  target_include_directories(costmap_unit_tests PRIVATE "include")
  target_include_directories(fusion_policy_unit_tests PRIVATE "include")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
