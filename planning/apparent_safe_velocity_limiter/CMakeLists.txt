cmake_minimum_required(VERSION 3.5)
project(apparent_safe_velocity_limiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

ament_auto_add_library(apparent_safe_velocity_limiter_node SHARED
  DIRECTORY src
)

ament_target_dependencies(apparent_safe_velocity_limiter_node PCL)
target_include_directories(apparent_safe_velocity_limiter_node
  SYSTEM PUBLIC
    "${PCL_INCLUDE_DIRS}"
)

# Disable warnings due to external dependencies
get_target_property(lanelet2_core_INCLUDE_DIR
  lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES
)

rclcpp_components_register_node(apparent_safe_velocity_limiter_node
  PLUGIN "apparent_safe_velocity_limiter::ApparentSafeVelocityLimiterNode"
  EXECUTABLE apparent_safe_velocity_limiter
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_forward_projection.cpp
    test/test_obstacles.cpp
    test/test_collision_distance.cpp
    test/test_occupancy_grid_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    apparent_safe_velocity_limiter_node
  )
endif()

add_executable(collision_benchmark
  benchmarks/collision_checker_benchmark.cpp
)
target_link_libraries(collision_benchmark
  apparent_safe_velocity_limiter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    script
)
