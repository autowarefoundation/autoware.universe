cmake_minimum_required(VERSION 3.14)
project(detected_object_feature_remover)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(detected_object_feature_remover_node SHARED
  src/detected_object_feature_remover.cpp
)

ament_auto_add_executable(${PROJECT_NAME}
  src/detected_object_feature_remover_node.cpp
)

target_link_libraries(${PROJECT_NAME}
detected_object_feature_remover_node glog
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
