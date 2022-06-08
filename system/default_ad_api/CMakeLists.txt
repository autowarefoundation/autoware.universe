cmake_minimum_required(VERSION 3.14)
project(default_ad_api)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface_version.cpp
)

ament_auto_add_library(motion_factor_aggregator_node SHARED
  src/awapi_autoware_util.cpp
  src/motion_factor_aggregator.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(motion_factor_aggregator_node PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_nodes(${PROJECT_NAME} "default_ad_api::InterfaceVersionNode")
rclcpp_components_register_nodes(motion_factor_aggregator_node "default_ad_api::MotionFactorAggregator")

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch test)
