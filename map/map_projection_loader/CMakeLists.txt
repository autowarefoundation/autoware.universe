cmake_minimum_required(VERSION 3.14)
project(map_projection_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(map_projection_loader
  src/map_projection_loader_node.cpp
  src/map_projection_loader.cpp
)
ament_target_dependencies(map_projection_loader)

target_link_libraries(map_projection_loader yaml-cpp)

if(BUILD_TESTING)
  add_launch_test(
    test/test_map_projection_loader_from_yaml.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
