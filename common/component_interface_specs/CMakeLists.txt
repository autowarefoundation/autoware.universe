cmake_minimum_required(VERSION 3.14)
project(component_interface_specs)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcl REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED) 
find_package(tier4_control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(tier4_map_msgs REQUIRED)
include_directories(
  include
  SYSTEM
  ${rclcpp_INCLUDE_DIRS}
  ${rosidl_runtime_cpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${autoware_adapi_v1_msgs_INCLUDE_DIRS}
  ${autoware_auto_planning_msgs_INCLUDE_DIRS}
  ${autoware_planning_msgs_INCLUDE_DIRS}
  ${tier4_control_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tier4_system_msgs_INCLUDE_DIRS}
  ${tier4_vehicle_msgs_INCLUDE_DIRS}
  ${autoware_auto_perception_msgs_INCLUDE_DIRS}
  ${tier4_map_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_component_interface_specs
    test/gtest_main.cpp 
    test/test_planning.cpp
    test/test_control.cpp
    test/test_localization.cpp
    test/test_system.cpp
    test/test_map.cpp
    test/test_perception.cpp
    test/test_vehicle.cpp
  )
  target_include_directories(test_component_interface_specs
    PRIVATE include
  )
endif()


ament_auto_package()
