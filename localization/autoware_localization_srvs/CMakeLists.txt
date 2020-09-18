cmake_minimum_required(VERSION 3.0.2)
project(autoware_localization_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_service_files(
  FILES
  PoseWithCovarianceStamped.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
