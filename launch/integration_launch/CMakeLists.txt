cmake_minimum_required(VERSION 2.8.3)
project(integration_launch)

find_package(catkin REQUIRED COMPONENTS
  autoware_launch
)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
