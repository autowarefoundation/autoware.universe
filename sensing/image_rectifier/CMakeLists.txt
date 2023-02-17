cmake_minimum_required(VERSION 3.14)
project(image_rectifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(image_rectifier SHARED
  src/image_rectifier.cpp
)

target_link_libraries(image_rectifier
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(image_rectifier
  PLUGIN "image_preprocessor::ImageRectifier"
  EXECUTABLE image_rectifier_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
