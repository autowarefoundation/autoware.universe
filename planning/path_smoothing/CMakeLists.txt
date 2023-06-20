cmake_minimum_required(VERSION 3.14)
project(path_smoothing)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(path_smoothing SHARED
  # node
  src/path_smoother_node.cpp
  # algorithms
  src/elastic_band.cpp
  # utils
  src/utils/trajectory_utils.cpp
)

target_include_directories(path_smoothing
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

# register node
rclcpp_components_register_node(path_smoothing
  PLUGIN "path_smoothing::ElasticBand"
  EXECUTABLE elastic_band_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
