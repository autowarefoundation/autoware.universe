cmake_minimum_required(VERSION 3.14)
project(hazard_status_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/converter.cpp
  src/merger.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "hazard_status_converter::Converter"
  "hazard_status_converter::Merger"
)

ament_auto_package(INSTALL_TO_SHARE config launch)
