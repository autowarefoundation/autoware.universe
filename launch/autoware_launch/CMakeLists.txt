cmake_minimum_required(VERSION 2.8.3)
project(autoware_launch)

find_package(catkin REQUIRED COMPONENTS
  vehicle_launch
  perception_launch
  sensing_launch
)

catkin_package()

install(
  DIRECTORY
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
