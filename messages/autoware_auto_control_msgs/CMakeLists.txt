# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_control_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AckermannControlCommand.idl"
  "msg/AckermannLateralCommand.idl"
  "msg/HighLevelControlCommand.idl"
  "msg/LongitudinalCommand.idl"
  DEPENDENCIES
    "builtin_interfaces"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
