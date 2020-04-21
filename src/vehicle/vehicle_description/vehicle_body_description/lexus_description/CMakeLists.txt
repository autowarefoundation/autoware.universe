cmake_minimum_required(VERSION 2.8.3)
project(lexus_description)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
    launch
    mesh
    urdf
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
