cmake_minimum_required(VERSION 3.14)
project(behavior_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

# Common
ament_auto_add_library(scene_module_lib SHARED
  src/scene_module/scene_module_bt_node_interface.cpp
  src/scene_module/utils/geometric_parallel_parking.cpp
  src/scene_module/utils/occupancy_grid_based_collision_detector.cpp
  src/scene_module/utils/path_shifter.cpp
  src/scene_module/scene_module_visitor.cpp
  src/utilities.cpp
  src/path_utilities.cpp
  src/debug_utilities.cpp
  src/steering_factor_interface.cpp
  src/turn_signal_decider.cpp
)

target_include_directories(scene_module_lib SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(scene_module_lib
  ${OpenCV_LIBRARIES}
)

target_include_directories(scene_module_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(scene_module_lib
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(scene_module_lib ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

# Scene Module: Avoidance
ament_auto_add_library(scene_module_avoidance SHARED
  src/scene_module/avoidance/avoidance_module.cpp
  src/scene_module/avoidance/avoidance_utils.cpp
  src/scene_module/avoidance/debug.cpp
)

target_include_directories(scene_module_avoidance
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_avoidance ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

# Scene Module: Lane Change
ament_auto_add_library(scene_module_lane_change SHARED
  src/scene_module/lane_change/external_request_lane_change_module.cpp
  src/scene_module/lane_change/lane_change_module.cpp
  src/scene_module/lane_change/util.cpp
  src/scene_module/lane_change/debug.cpp
)

target_include_directories(scene_module_lane_change
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_lane_change ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_lane_change scene_module_lib)

# Scene Module: Lane Following
ament_auto_add_library(scene_module_lane_following SHARED
  src/scene_module/lane_following/lane_following_module.cpp
)

target_include_directories(scene_module_lane_following
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_lane_following ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_lane_following scene_module_lib)

# Scene Module: Pull Out
ament_auto_add_library(scene_module_pull_out SHARED
  src/scene_module/pull_out/pull_out_module.cpp
  src/scene_module/pull_out/util.cpp
  src/scene_module/pull_out/shift_pull_out.cpp
  src/scene_module/pull_out/geometric_pull_out.cpp
)

target_include_directories(scene_module_pull_out
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_pull_out ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_pull_out scene_module_lib)

# Scene Module: Pull Over
ament_auto_add_library(scene_module_pull_over SHARED
  src/scene_module/pull_over/pull_over_module.cpp
  src/scene_module/pull_over/util.cpp
  src/scene_module/pull_over/shift_pull_over.cpp
  src/scene_module/pull_over/geometric_pull_over.cpp
  src/scene_module/pull_over/goal_searcher.cpp
)

target_include_directories(scene_module_pull_over
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_pull_over ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_pull_over scene_module_lib)

# Scene Module: Side Shift
ament_auto_add_library(scene_module_side_shift SHARED
  src/scene_module/side_shift/side_shift_module.cpp
  src/scene_module/side_shift/util.cpp
)

target_include_directories(scene_module_side_shift
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scene_module_side_shift ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(scene_module_side_shift scene_module_lib)

# Node
ament_auto_add_library(behavior_path_planner SHARED
  src/behavior_path_planner_node.cpp
  src/behavior_tree_manager.cpp
)

target_include_directories(behavior_path_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_path_planner ${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

target_link_libraries(behavior_path_planner
  scene_module_lib
  scene_module_avoidance
  scene_module_lane_change
  scene_module_lane_following
  scene_module_pull_out
  scene_module_pull_over
  scene_module_side_shift
  )

ament_export_dependencies(${BEHAVIOR_PATH_PLANNER_DEPENDENCIES})

rclcpp_components_register_node(behavior_path_planner_node
  PLUGIN "behavior_path_planner::BehaviorPathPlannerNode"
  EXECUTABLE behavior_path_planner
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_utilities
    test/input.cpp
    test/test_utilities.cpp
  )
  target_link_libraries(test_${CMAKE_PROJECT_NAME}_utilities
    behavior_path_planner_node
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_avoidance_module
    test/test_avoidance_utils.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_avoidance_module
    behavior_path_planner_node
  )

  ament_add_ros_isolated_gmock(test_${CMAKE_PROJECT_NAME}_turn_signal
    test/test_turn_signal.cpp
  )

  target_link_libraries(test_${CMAKE_PROJECT_NAME}_turn_signal
    behavior_path_planner_node
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
