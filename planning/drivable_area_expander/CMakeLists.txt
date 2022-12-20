cmake_minimum_required(VERSION 3.5)
project(drivable_area_expander)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(drivable_area_expander_node SHARED
  DIRECTORY src
)

# Disable warnings due to external dependencies
get_target_property(lanelet2_core_INCLUDE_DIR
  lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES
)

rclcpp_components_register_node(drivable_area_expander_node
  PLUGIN "drivable_area_expander::DrivableAreaExpanderNode"
  EXECUTABLE drivable_area_expander
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_footprint.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    drivable_area_expander_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
