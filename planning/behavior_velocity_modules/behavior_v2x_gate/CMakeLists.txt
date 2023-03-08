cmake_minimum_required(VERSION 3.14)
project(behavior_v2x_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
)

# TODO(Takagi, Isamu): remove unused dependency
find_package(PCL REQUIRED COMPONENTS common)
ament_target_dependencies(${PROJECT_NAME} PCL)

pluginlib_export_plugin_description_file(behavior_velocity_planner plugins.xml)

ament_auto_package()
