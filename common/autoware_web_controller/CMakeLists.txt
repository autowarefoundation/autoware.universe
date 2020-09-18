cmake_minimum_required(VERSION 3.0.2)
project(autoware_web_controller)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
