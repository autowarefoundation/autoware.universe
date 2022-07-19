cmake_minimum_required(VERSION 3.14)
project(mrm_sudden_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mrm_sudden_stop_operator_component SHARED
  src/mrm_sudden_stop_operator/mrm_sudden_stop_operator_core.cpp
)

rclcpp_components_register_node(mrm_sudden_stop_operator_component
  PLUGIN "mrm_sudden_stop_operator::MRMSuddenStopOperator"
  EXECUTABLE mrm_sudden_stop_operator)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
