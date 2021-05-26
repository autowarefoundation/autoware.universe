
cmake_minimum_required(VERSION 3.5)
project(naive_path_prediction)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(naive_path_prediction SHARED
  src/node.cpp
)

rclcpp_components_register_node(naive_path_prediction
  PLUGIN "NaivePathPredictionNode"
  EXECUTABLE naive_path_prediction_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
