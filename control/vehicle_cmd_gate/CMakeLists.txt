cmake_minimum_required(VERSION 3.5)
project(vehicle_cmd_gate)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Generate exe file
ament_auto_add_library(vehicle_cmd_gate_node SHARED
  src/vehicle_cmd_gate.cpp
  src/vehicle_cmd_filter.cpp
)

rclcpp_components_register_node(vehicle_cmd_gate_node
  PLUGIN "VehicleCmdGate"
  EXECUTABLE vehicle_cmd_gate
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
