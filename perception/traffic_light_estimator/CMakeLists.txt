cmake_minimum_required(VERSION 3.5)
project(traffic_light_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

###########
## Build ##
###########

include_directories()

ament_auto_add_library(traffic_light_estimator SHARED
  src/node.cpp
)

rclcpp_components_register_node(traffic_light_estimator
  PLUGIN "traffic_light::TrafficLightEstimatorNode"
  EXECUTABLE traffic_light_estimator_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
