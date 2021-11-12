cmake_minimum_required(VERSION 3.5)
project(freespace_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(FREESPACE_PLANNER_NODE_LIB freespace_planner_node)
ament_auto_add_library(${FREESPACE_PLANNER_NODE_LIB} SHARED
  src/freespace_planner/freespace_planner_node.cpp)
autoware_set_compile_options(${FREESPACE_PLANNER_NODE_LIB})


rclcpp_components_register_node(${FREESPACE_PLANNER_NODE_LIB}
  PLUGIN "autoware::planning::freespace_planner::FreespacePlannerNode"
  EXECUTABLE ${FREESPACE_PLANNER_NODE_LIB}_exe
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# 
#   find_package(autoware_testing REQUIRED)
#   add_smoke_test(${PROJECT_NAME} ${FREESPACE_PLANNER_NODE_LIB}_exe)
# 
#   add_ros_test(
#     test/freespace_planner.test.py
#     TIMEOUT "15"
#   )
# endif()

ament_auto_package(INSTALL_TO_SHARE launch param)
