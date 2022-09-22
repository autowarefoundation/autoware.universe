cmake_minimum_required(VERSION 3.14)
project(static_path_smoother)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# node
ament_auto_add_library(static_path_smoother SHARED
  src/node.cpp
)

target_include_directories(static_path_smoother
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(static_path_smoother
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(static_path_smoother
  PLUGIN "StaticPathSmoother"
  EXECUTABLE static_path_smoother_node
)

# main
ament_auto_add_library(main SHARED
  src/main.cpp
)

target_include_directories(main
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(main
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(main
  PLUGIN "StaticPathSmoother"
  EXECUTABLE main
)
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
