cmake_minimum_required(VERSION 3.14)
project(lidar_centerpoint_tvm)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(tvm_runtime_DIR ${tvm_vendor_DIR})
find_package(tvm_runtime CONFIG REQUIRED)

# Get target backend
set(${PROJECT_NAME}_BACKEND llvm CACHE STRING "${PROJECT_NAME} neural network backend")

# Test if files exist. The results are set in centerpoint_ve_FOUND and centerpoint_bnh_FOUND
autoware_check_neural_network(centerpoint_ve "${${PROJECT_NAME}_BACKEND}")
autoware_check_neural_network(centerpoint_bnh "${${PROJECT_NAME}_BACKEND}")

if(centerpoint_ve_FOUND AND centerpoint_bnh_FOUND)
  ## the shared settings
  include_directories(
    include
  )

  ## centerpoint_tvm ##
  ament_auto_add_library(centerpoint_tvm_lib SHARED
    lib/centerpoint_tvm.cpp
    lib/utils.cpp
    lib/ros_utils.cpp
    lib/network/scatter.cpp
    lib/preprocess/pointcloud_densification.cpp
    lib/preprocess/voxel_generator.cpp
    lib/preprocess/generate_features.cpp
    lib/postprocess/circle_nms.cpp
    lib/postprocess/generate_detected_boxes.cpp
  )

  target_compile_options(centerpoint_tvm_lib PRIVATE "-Wno-sign-conversion" "-Wno-conversion")
  target_compile_definitions(centerpoint_tvm_lib PRIVATE NETWORKS_BACKEND=${${PROJECT_NAME}_BACKEND})

  target_include_directories(centerpoint_tvm_lib SYSTEM PUBLIC
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include>
    "${tvm_vendor_INCLUDE_DIRS}"
  )

  target_link_libraries(centerpoint_tvm_lib
    ${tvm_runtime_LIBRARIES}
  )

  ## node ##
  ament_auto_add_library(lidar_centerpoint_tvm_component SHARED
    src/node.cpp
  )

  target_link_libraries(lidar_centerpoint_tvm_component
    centerpoint_tvm_lib
  )

  rclcpp_components_register_node(lidar_centerpoint_tvm_component
    PLUGIN "autoware::perception::lidar_centerpoint_tvm::LidarCenterPointTVMNode"
    EXECUTABLE lidar_centerpoint_tvm_node
  )

  ament_auto_package(INSTALL_TO_SHARE
    launch
    config
  )
else()
  message(WARNING "Neural network not found, skipping package.")
endif()
