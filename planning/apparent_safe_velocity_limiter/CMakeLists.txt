cmake_minimum_required(VERSION 3.5)
project(apparent_safe_velocity_limiter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

ament_auto_add_library(apparent_safe_velocity_limiter_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(apparent_safe_velocity_limiter_node
  PLUGIN "apparent_safe_velocity_limiter::ApparentSafeVelocityLimiterNode"
  EXECUTABLE apparent_safe_velocity_limiter
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_collision_distance.cpp
    test/test_occupancy_grid_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    apparent_safe_velocity_limiter_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    script
)
