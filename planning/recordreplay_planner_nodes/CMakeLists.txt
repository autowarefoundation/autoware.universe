cmake_minimum_required(VERSION 3.14)
project(recordreplay_planner_nodes)

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

target_compile_options(${RECORDREPLAY_PLANNER_NODE_LIB} PRIVATE -Wno-error=old-style-cast)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit test
  ament_add_gtest(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests
    test/gtest_main.cpp
    test/sanity_checks.cpp)
  target_link_libraries(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests ${RECORDREPLAY_PLANNER_NODE_LIB})
  target_include_directories(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests PRIVATE "include")
endif()

ament_auto_package(INSTALL_TO_SHARE param launch)
