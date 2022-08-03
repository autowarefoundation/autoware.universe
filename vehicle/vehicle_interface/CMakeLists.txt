cmake_minimum_required(VERSION 3.14)
project(vehicle_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(INTERFACE_NODE vehicle_interface_node)
ament_auto_add_library(${INTERFACE_NODE} SHARED
  include/vehicle_interface/base_interface.hpp
  include/vehicle_interface/vehicle_interface_node.hpp
  src/vehicle_interface_node.cpp
  src/base_interface.cpp
)

if(BUILD_TESTING)
  set(INTERFACE_NODE_TEST test_vehicle_interface_node)
  ament_add_ros_isolated_gtest(${INTERFACE_NODE_TEST}
    test/test_vehicle_interface.cpp
    test/dummy_node.hpp
    test/dummy_node.cpp
  )
  ament_target_dependencies(${INTERFACE_NODE_TEST} fake_test_node)
  target_link_libraries(
    ${INTERFACE_NODE_TEST} ${INTERFACE_NODE})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  param
)
