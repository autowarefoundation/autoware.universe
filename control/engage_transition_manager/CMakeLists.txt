cmake_minimum_required(VERSION 3.14)
project(engage_transition_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(engage_transition_manager_node SHARED
  src/engage_transition_manager.cpp
  src/state.cpp
  src/data.cpp
)

rclcpp_components_register_node(engage_transition_manager_node
  PLUGIN "engage_transition_manager::EngageTransitionManager"
  EXECUTABLE engage_transition_manager_exe
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/EngageTransitionManagerDebug.msg"
  DEPENDENCIES builtin_interfaces
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(engage_transition_manager_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(engage_transition_manager_node "${cpp_typesupport_target}")
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
