cmake_minimum_required(VERSION 3.14)
project(trajectory_follower_f1tenth)

find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

autoware_package()

set(CONTROLLER_NODE controller_node)
ament_auto_add_library(${CONTROLLER_NODE} SHARED
  include/trajectory_follower_f1tenth/controller_node.hpp
  src/controller_node.cpp
)

target_compile_options(${CONTROLLER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_node::Controller"
  EXECUTABLE ${CONTROLLER_NODE}_exe
)

# f1tenth trajectory follower
set(F1TENTH_TRAJECTORY_FOLLOWER_NODE f1tenth_trajectory_follower)
ament_auto_add_library(${F1TENTH_TRAJECTORY_FOLLOWER_NODE} SHARED
  include/trajectory_follower_f1tenth/f1tenth_trajectory_follower.hpp
  src/f1tenth_trajectory_follower.cpp
)

ament_target_dependencies(${F1TENTH_TRAJECTORY_FOLLOWER_NODE} ackermann_msgs)

# TODO(f1tenth_trajectory_follower) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(f1tenth_trajectory_follower) : Temporary workaround until this is fixed.
target_compile_options(${F1TENTH_TRAJECTORY_FOLLOWER_NODE} PRIVATE -Wno-error=old-style-cast)
rclcpp_components_register_node(${F1TENTH_TRAJECTORY_FOLLOWER_NODE}
  PLUGIN "f1tenth_trajectory_follower::F1tenthTrajectoryFollower"
  EXECUTABLE ${F1TENTH_TRAJECTORY_FOLLOWER_NODE}_exe
)

if(BUILD_TESTING)
  set(TRAJECTORY_FOLLOWER_NODES_TEST test_trajectory_follower_nodes)
  ament_add_ros_isolated_gtest(${TRAJECTORY_FOLLOWER_NODES_TEST}
    test/trajectory_follower_test_utils.hpp
    test/test_controller_node.cpp
  )
  ament_target_dependencies(${TRAJECTORY_FOLLOWER_NODES_TEST} fake_test_node)
  target_link_libraries(
    ${TRAJECTORY_FOLLOWER_NODES_TEST} ${CONTROLLER_NODE})

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${CONTROLLER_NODE}_exe
    PARAM_FILENAMES "lateral_controller_defaults.param.yaml longitudinal_controller_defaults.param.yaml
test_vehicle_info.param.yaml test_nearest_search.param.yaml"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
)
