cmake_minimum_required(VERSION 3.22)
project(autoware_planning_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(pluginlib REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_add_library(planning_evaluator_node SHARED
  DIRECTORY src
)

rosidl_generate_interfaces(
        ${PROJECT_NAME}
        "msg/Metric.msg"
        "msg/MetricList.msg"
        DEPENDENCIES builtin_interfaces
)

# For using message definitions from the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(planning_evaluator_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(planning_evaluator_node "${cpp_typesupport_target}")
endif()

rclcpp_components_register_node(planning_evaluator_node
  PLUGIN "planning_diagnostics::PlanningEvaluatorNode"
  EXECUTABLE planning_evaluator
)

rclcpp_components_register_node(planning_evaluator_node
  PLUGIN "planning_diagnostics::MotionEvaluatorNode"
  EXECUTABLE motion_evaluator
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_planning_evaluator
    test/test_planning_evaluator_node.cpp
  )
  target_link_libraries(test_planning_evaluator
    planning_evaluator_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
