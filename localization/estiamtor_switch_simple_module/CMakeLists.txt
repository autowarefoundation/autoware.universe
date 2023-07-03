cmake_minimum_required(VERSION 3.14)
project(estimator_switch_simple_module)

find_package(autoware_cmake REQUIRED)
autoware_package()

pluginlib_export_plugin_description_file(pose_estimator_manager plugins.xml)

ament_auto_add_library(${PROJECT_NAME}
  SHARED
  src/simple_switch_rule.cpp
)

ament_auto_package(INSTALL_TO_SHARE)
