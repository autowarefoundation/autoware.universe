# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_system_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutowareState.idl"
  "msg/ControlDiagnostic.idl"
  "msg/DiagnosticHeader.idl"
  "msg/DrivingCapability.idl"
  "msg/EmergencyState.idl"
  "msg/Float32MultiArrayDiagnostic.idl"
  "msg/HazardStatus.idl"
  "msg/HazardStatusStamped.idl"
  DEPENDENCIES
    "builtin_interfaces"
    "diagnostic_msgs"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
