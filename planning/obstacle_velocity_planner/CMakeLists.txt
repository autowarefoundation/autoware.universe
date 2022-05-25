cmake_minimum_required(VERSION 3.8)
project(obstacle_velocity_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(obstacle_velocity_planner_core SHARED
  src/node.cpp
  src/utils.cpp
  src/polygon_utils.cpp
  src/optimization_based_planner/resample.cpp
  src/optimization_based_planner/box2d.cpp
  src/optimization_based_planner/velocity_optimizer.cpp
  src/optimization_based_planner/optimization_based_planner.cpp
  src/rule_based_planner/rule_based_planner.cpp
)

rclcpp_components_register_node(obstacle_velocity_planner_core
  PLUGIN "motion_planning::ObstacleVelocityPlannerNode"
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
