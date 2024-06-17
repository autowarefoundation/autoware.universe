cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_msgs)
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs)

add_message_files(
  FILES
  PolygonArray.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs
)
