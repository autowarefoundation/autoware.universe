cmake_minimum_required(VERSION 3.14)
project(accel_brake_map_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(accel_brake_map_calibrator_node
  src/accel_brake_map_calibrator_node.cpp
  src/utility.cpp
)

# to use same package defined message
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/AccelBrakeMapCalibratorInfo.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(
        cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(accel_brake_map_calibrator_node "${cpp_typesupport_target}")

rclcpp_components_register_node(accel_brake_map_calibrator_node
  PLUGIN "accel_brake_map_calibrator::AccelBrakeMapCalibrator"
  EXECUTABLE accel_brake_map_calibrator_exe
)

install(
  PROGRAMS
  scripts/__init__.py
  scripts/actuation_cmd_publisher.py
  scripts/accel_tester.py
  scripts/brake_tester.py
  scripts/config.py
  scripts/delay_estimator.py
  scripts/plotter.py
  scripts/view_statistics.py
  scripts/calc_utils.py
  scripts/csv_reader.py
  scripts/log_analyzer.py
  scripts/view_plot.py
  scripts/new_accel_brake_map_server.py
  test/plot_csv_client_node.py
  test/sim_actuation_status_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz)
