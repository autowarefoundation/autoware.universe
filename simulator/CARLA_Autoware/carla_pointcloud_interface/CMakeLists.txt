cmake_minimum_required(VERSION 3.5)
project(carla_pointcloud_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

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


find_package(Boost COMPONENTS signals)
find_package(PCL REQUIRED COMPONENTS common)



find_package(PkgConfig REQUIRED)

include_directories(
  ${PCL_COMMON_INCLUDE_DIRS}
)

ament_auto_add_library(carla_pointcloud_interface SHARED
  src/carla_pointcloud_interface/carla_pointcloud_interface_node.cpp
)


# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(carla_pointcloud_interface PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

rclcpp_components_register_node(carla_pointcloud_interface
  PLUGIN "PointCloudInterface"
  EXECUTABLE carla_pointcloud_interface_node
)


ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
