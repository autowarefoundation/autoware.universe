cmake_minimum_required(VERSION 3.8)
project(carla_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs_py REQUIRED)
find_package(datetime REQUIRED)

find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(transform3d REQUIRED)
find_package(math REQUIRED)
find_package(numpy REQUIRED)
find_package(carla_data_provider REQUIRED)




find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

ament_export_dependencies(rclpy)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


ament_auto_package(
  launch
  resource
  src
)
ament_package()
