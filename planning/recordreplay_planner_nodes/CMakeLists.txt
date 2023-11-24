cmake_minimum_required(VERSION 3.14)

project(recordreplay_planner_nodes)

### Dependencies
find_package(autoware_cmake REQUIRED)

autoware_package()

set(RECORDREPLAY_PLANNER_NODE_LIB recordreplay_planner_node)

ament_auto_add_library(${RECORDREPLAY_PLANNER_NODE_LIB} SHARED
  src/recordreplay_planner_nodes/recordreplay_planner_node.cpp)

set(NODE_NAME ${RECORDREPLAY_PLANNER_NODE_LIB}_exe)

rclcpp_components_register_node(${RECORDREPLAY_PLANNER_NODE_LIB}
  PLUGIN "motion::planning::recordreplay_planner_nodes::RecordReplayPlannerNode"
  EXECUTABLE ${NODE_NAME}
)

ament_auto_package(INSTALL_TO_SHARE param launch)
