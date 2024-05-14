cmake_minimum_required(VERSION 3.14)
project(system_error_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(system_error_monitor_component SHARED
  src/system_error_monitor_core.cpp
)

rclcpp_components_register_node(system_error_monitor_component
  PLUGIN "AutowareErrorMonitor"
  EXECUTABLE system_error_monitor)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
