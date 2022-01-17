cmake_minimum_required(VERSION 3.5)
project(planning_manager)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanningData.idl"
  "srv/BehaviorPathPlannerPlan.idl"
  "srv/BehaviorPathPlannerValidate.idl"
  "srv/BehaviorVelocityPlannerPlan.idl"
  "srv/BehaviorVelocityPlannerValidate.idl"
  DEPENDENCIES
    "autoware_auto_planning_msgs"
    "autoware_auto_perception_msgs"
    "tier4_planning_msgs"
    "nav_msgs"
    "std_msgs"
)

ament_auto_add_library(planning_manager_node SHARED
  src/planning_manager_node.cpp
)

rosidl_target_interfaces(planning_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(planning_manager_node
  PLUGIN "planning_manager::PlanningManagerNode"
  EXECUTABLE planning_manager_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    # config
    # launch
)
