cmake_minimum_required(VERSION 3.8)
project(lgsvl_ground_truth)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/odom_ground_truth.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

set(NODE_NAME ${PROJECT_NAME}_exe)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lgsvl_ground_truth::OdomGroundTruth"
  EXECUTABLE ${NODE_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
