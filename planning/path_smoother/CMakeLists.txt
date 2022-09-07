cmake_minimum_required(VERSION 3.14)
project(path_smoother)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(path_smoother SHARED
  src/node.cpp
)

target_include_directories(path_smoother
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(path_smoother
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(path_smoother
  PLUGIN "PathSmoother"
  EXECUTABLE path_smoother_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
