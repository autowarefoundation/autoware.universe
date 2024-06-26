cmake_minimum_required(VERSION 3.5)
project(simple_object_merger)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

# Targets
ament_auto_add_library(simple_object_merger_node_component SHARED
  src/simple_object_merger_node/simple_object_merger_node.cpp
)

rclcpp_components_register_node(simple_object_merger_node_component
  PLUGIN "simple_object_merger::SimpleObjectMergerNode"
  EXECUTABLE simple_object_merger_node
)

# Tests
if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_simple_object_merger_node ${test_files})

  target_link_libraries(test_simple_object_merger_node
    simple_object_merger_node_component
  )
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
