# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_planning_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlannerCostmap.idl"
  "action/PlanTrajectory.idl"
  "action/RecordTrajectory.idl"
  "action/ReplayTrajectory.idl"
  "msg/HADMapRoute.idl"
  "msg/OrderMovement.idl"
  "msg/Route.idl"
  "msg/Trajectory.idl"
  "msg/TrajectoryPoint.idl"
  "msg/Path.idl"
  "msg/PathPoint.idl"
  "msg/PathWithLaneId.idl"
  "msg/PathPointWithLaneId.idl"
  "srv/ModifyTrajectory.idl"
  DEPENDENCIES
    "autoware_auto_geometry_msgs"
    "autoware_auto_mapping_msgs"
    "builtin_interfaces"
    "geometry_msgs"
    "nav_msgs"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
