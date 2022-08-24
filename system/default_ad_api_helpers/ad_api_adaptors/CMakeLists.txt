cmake_minimum_required(VERSION 3.14)
project(ad_api_adaptors)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/routing_adaptor.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "ad_api_adaptors::RoutingAdaptor"
)

ament_auto_package(INSTALL_TO_SHARE launch)
