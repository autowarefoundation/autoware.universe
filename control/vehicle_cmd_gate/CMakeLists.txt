cmake_minimum_required(VERSION 3.5)
project(vehicle_cmd_gate)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

### Dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Add path of include dir
include_directories(include)

# Generate exe file
add_executable(vehicle_cmd_gate src/vehicle_cmd_gate_node.cpp src/vehicle_cmd_gate.cpp src/vehicle_cmd_filter.cpp)

set(VEHICLE_CMD_GATE_DEPENDENCIES
	autoware_control_msgs
	autoware_vehicle_msgs
	rclcpp
)
ament_target_dependencies(vehicle_cmd_gate ${VEHICLE_CMD_GATE_DEPENDENCIES})
ament_export_dependencies(${VEHICLE_CMD_GATE_DEPENDENCIES})

install(
  TARGETS vehicle_cmd_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
