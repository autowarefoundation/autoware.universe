cmake_minimum_required(VERSION 3.14)
project(localizer_switch_rule_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

pluginlib_export_plugin_description_file(pose_estimator_manager plugins.xml)

ament_auto_add_library(${PROJECT_NAME}
  SHARED
  src/pcd_occupancy_rule.cpp
)

ament_auto_package(INSTALL_TO_SHARE)
