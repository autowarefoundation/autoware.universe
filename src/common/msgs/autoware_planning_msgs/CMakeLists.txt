cmake_minimum_required(VERSION 2.8.3)
project(autoware_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    LaneSequence.msg
    Path.msg
    PathPoint.msg
    PathWithLaneId.msg
    PathPointWithLaneId.msg
    Route.msg
    RouteSection.msg
    Scenario.msg
    Trajectory.msg
    TrajectoryPoint.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    nav_msgs
)
