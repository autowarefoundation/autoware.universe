cmake_minimum_required(VERSION 3.8)
project(component_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED COMPONENTS
  filesystem
#  thread
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/component_monitor.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::component_monitor::ComponentMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
#  config
  launch
)
