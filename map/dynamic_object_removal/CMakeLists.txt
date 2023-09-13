cmake_minimum_required(VERSION 3.5)
project(dynamic_object_removal)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

# Ensure that the "include" folder is added to the include path
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add the executable for your node
ament_auto_add_executable(dynamic_object_removal
  src/dynamic_object_removal.cpp)

# Link the necessary libraries to your executable
ament_target_dependencies(dynamic_object_removal
  rclcpp
  sensor_msgs
  autoware_auto_perception_msgs
  message_filters
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
)

target_include_directories(dynamic_object_removal
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dynamic_object_removal ${PCL_LIBRARIES})

# Install the executable to the "lib" folder of your package
install(TARGETS 
dynamic_object_removal
DESTINATION install/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)