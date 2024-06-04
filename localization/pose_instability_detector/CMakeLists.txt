cmake_minimum_required(VERSION 3.14)
project(pose_instability_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(src)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_instability_detector.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "PoseInstabilityDetector"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test.cpp
    src/pose_instability_detector.cpp
  )
endif()

install(
  DIRECTORY src/
  DESTINATION include/pose_instability_detector
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_include_directories(include)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
