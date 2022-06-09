cmake_minimum_required(VERSION 3.14)
project(engage_transition_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(engage_transition_manager_node SHARED
  src/engage_transition_manager.cpp
  src/state.cpp
  src/data.cpp
)

rclcpp_components_register_node(engage_transition_manager_node
  PLUGIN "engage_transition_manager::EngageTransitionManager"
  EXECUTABLE engage_transition_manager_exe
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
