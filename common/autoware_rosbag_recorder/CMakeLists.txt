cmake_minimum_required(VERSION 3.0.2)
project(autoware_rosbag_recorder)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
