
cmake_minimum_required(VERSION 3.5)
project(naive_path_prediction)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(naive_path_prediction_node src/node.cpp)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
