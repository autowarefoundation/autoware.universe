cmake_minimum_required(VERSION 3.5)
project(carla_gnss_interface)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types )
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

ament_auto_add_library(carla_gnss_interface SHARED
  src/carla_gnss_interface/carla_gnss_interface_node.cpp
)

target_link_libraries(carla_gnss_interface ${PROJ_LIBRARIES})

rclcpp_components_register_node(carla_gnss_interface
  PLUGIN "GnssInterface"
  EXECUTABLE carla_gnss_interface_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
