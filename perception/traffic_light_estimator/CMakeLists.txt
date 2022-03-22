cmake_minimum_required(VERSION 3.5)
project(traffic_light_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Werror)
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

include_directories()

ament_auto_add_library(traffic_light_estimator SHARED
  src/node.cpp
)

rclcpp_components_register_node(traffic_light_estimator
  PLUGIN "traffic_light::TrafficLightEstimatorNode"
  EXECUTABLE traffic_light_estimater_node
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
