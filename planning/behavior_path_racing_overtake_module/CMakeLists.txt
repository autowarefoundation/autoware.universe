cmake_minimum_required(VERSION 3.14)
project(behavior_path_racing_overtake_module)

find_package(autoware_cmake REQUIRED)

autoware_package()
pluginlib_export_plugin_description_file(behavior_path_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/manager.cpp
  src/state.cpp
  src/util.cpp
)

ament_auto_package(INSTALL_TO_SHARE config)

