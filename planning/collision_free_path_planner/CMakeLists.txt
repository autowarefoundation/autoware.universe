cmake_minimum_required(VERSION 3.14)
project(collision_free_path_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(collision_free_path_planner SHARED
  # node and core algorithms
  src/node.cpp
  src/replan_checker.cpp
  src/costmap_generator.cpp
  src/eb_path_optimizer.cpp
  src/mpt_optimizer.cpp
  # debug marker
  src/debug_marker.cpp
  # vehicle model
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  # utils
  src/utils/utils.cpp
  src/utils/cv_utils.cpp
)

target_include_directories(collision_free_path_planner
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(collision_free_path_planner
  ${OpenCV_LIBRARIES}
)

# register node
rclcpp_components_register_node(collision_free_path_planner
  PLUGIN "collision_free_path_planner::CollisionFreePathPlanner"
  EXECUTABLE collision_free_path_planner_node
)

# unit test
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_collision_free_path_planner
    test/test_mpt_optimizer.cpp
  )
  ament_target_dependencies(test_collision_free_path_planner fake_test_node)
  target_link_libraries(
    test_collision_free_path_planner collision_free_path_planner)

  find_package(autoware_testing REQUIRED)
  add_smoke_test(collision_free_path_planner collision_free_path_planner_node
    PARAM_FILENAMES "collision_free_path_planner.param.yaml test_vehicle_info.param.yaml test_nearest_search.param.yaml"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    scripts
    param
)
