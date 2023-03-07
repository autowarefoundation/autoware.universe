cmake_minimum_required(VERSION 3.14)
project(logging_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED src/logger_level.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME} "logging_utils::LoggerLevel")

ament_auto_package()
