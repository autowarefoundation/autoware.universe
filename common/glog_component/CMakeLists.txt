cmake_minimum_required(VERSION 3.14)
project(glog_component)

find_package(autoware_cmake REQUIRED)
autoware_package()


ament_auto_add_library(glog_component SHARED
  src/glog_component.cpp
)
target_link_libraries(glog_component glog)
target_include_directories(glog_component SYSTEM PUBLIC)

rclcpp_components_register_node(glog_component
  PLUGIN "GlogComponent"
  EXECUTABLE glog_component_node
)

ament_auto_package(INSTALL_TO_SHARE)
