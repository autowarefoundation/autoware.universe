cmake_minimum_required(VERSION 3.14)
project(cgroup_setter)

find_package(autoware_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/cgroup_setter.cpp
)

target_link_libraries(cgroup_setter yaml-cpp)
target_link_libraries(cgroup_setter ${Boost_LIBRARIES})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
