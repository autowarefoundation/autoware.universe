cmake_minimum_required(VERSION 3.5)
project(autoware_motion_velocity_obstacle_velocity_limiter_module)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_target_dependencies(${PROJECT_NAME} PCL)
target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    "${PCL_INCLUDE_DIRS}"
)

# Disable warnings due to external dependencies
get_target_property(lanelet2_core_INCLUDE_DIR
  lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES
)

pluginlib_export_plugin_description_file(autoware_motion_velocity_planner_node plugins.xml)

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
    ${PROJECT_NAME}
  )
endif()

add_executable(collision_benchmark
  benchmarks/collision_checker_benchmark.cpp
)
target_link_libraries(collision_benchmark
  ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    script
)
