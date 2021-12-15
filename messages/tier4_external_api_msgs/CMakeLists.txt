cmake_minimum_required(VERSION 3.5)
project(tier4_external_api_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ControlCommand.msg
  msg/ControlCommandStamped.msg
  msg/Emergency.msg
  msg/EngageStatus.msg
  msg/GearShift.msg
  msg/GearShiftStamped.msg
  msg/Heartbeat.msg
  msg/MapHash.msg
  msg/ResponseStatus.msg
  msg/TurnSignal.msg
  msg/TurnSignalStamped.msg
  srv/Engage.srv
  srv/InitializePose.srv
  srv/InitializePoseAuto.srv
  srv/SetEmergency.srv
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
