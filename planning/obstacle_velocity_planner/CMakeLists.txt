cmake_minimum_required(VERSION 3.8)
project(obstacle_velocity_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(obstacle_velocity_planner_core SHARED
  src/node.cpp
  src/resample.cpp
  src/box2d.cpp
  src/velocity_optimizer.cpp
)

rclcpp_components_register_node(obstacle_velocity_planner_core
  PLUGIN "ObstacleVelocityPlanner"
  EXECUTABLE obstacle_velocity_planner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

install(PROGRAMS
  scripts/trajectory_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)
