cmake_minimum_required(VERSION 3.5)
project(autoware_planning_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnableAvoidance.msg"
  "msg/IsAvoidancePossible.msg"
  "msg/LaneChangeCommand.msg"
  "msg/LaneChangeStatus.msg"
  "msg/LaneSequence.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/PathWithLaneId.msg"
  "msg/PathPointWithLaneId.msg"
  "msg/Route.msg"
  "msg/RouteSection.msg"
  "msg/Scenario.msg"
  "msg/StopFactor.msg"
  "msg/StopReason.msg"
  "msg/StopReasonArray.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/VelocityLimit.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_package()
