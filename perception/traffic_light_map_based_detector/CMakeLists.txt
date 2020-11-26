cmake_minimum_required(VERSION 3.5)
project(traffic_light_map_based_detector)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_executable(traffic_light_map_based_detector_node
  src/node.cpp
  src/main.cpp
)

#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
