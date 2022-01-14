cmake_minimum_required(VERSION 3.5)
project(planning_manager)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(planning_manager_node SHARED
  src/planning_manager_node.cpp
)

rclcpp_components_register_node(planning_manager_node
  PLUGIN "planning_manager::PlanningManagerNode"
  EXECUTABLE planning_manager
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    # config
    launch
)
