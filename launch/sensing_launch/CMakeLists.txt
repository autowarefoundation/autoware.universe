cmake_minimum_required(VERSION 2.8.3)
project(sensing_launch)

find_package(catkin REQUIRED)

# Force Tier IV's fork version
find_package(velodyne_driver 0.2.0 EXACT REQUIRED)
find_package(velodyne_pointcloud 0.2.0 EXACT REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
    data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
