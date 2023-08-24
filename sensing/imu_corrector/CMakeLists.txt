cmake_minimum_required(VERSION 3.14)
project(imu_corrector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(gyro_bias_estimation_module SHARED
  src/gyro_bias_estimation_module.cpp
)

ament_auto_add_library(imu_corrector_node SHARED
  src/imu_corrector_core.cpp
)

ament_auto_add_library(gyro_bias_validator_node SHARED
  src/gyro_bias_validator.cpp
)

target_link_libraries(gyro_bias_validator_node gyro_bias_estimation_module)

rclcpp_components_register_node(imu_corrector_node
  PLUGIN "imu_corrector::ImuCorrector"
  EXECUTABLE imu_corrector
)

rclcpp_components_register_node(gyro_bias_validator_node
  PLUGIN "imu_corrector::GyroBiasValidator"
  EXECUTABLE gyro_bias_validator
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
