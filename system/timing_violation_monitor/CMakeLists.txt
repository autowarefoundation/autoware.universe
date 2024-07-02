cmake_minimum_required(VERSION 3.8)
project(timing_violation_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/timing_violation_monitor_core.cpp
  src/timing_violation_monitor_debug.cpp
  src/timing_violation_monitor_node.cpp
)

ament_auto_add_library(timing_violation_monitor_utils SHARED
  src/utils/message_consumption_notifier.cpp
  include/timing_violation_monitor_utils/message_consumption_notifier.hpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_message_consumption_notifier
    test/test_message_consumption_notifier.cpp)
  target_link_libraries(test_message_consumption_notifier
    timing_violation_monitor_utils)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
