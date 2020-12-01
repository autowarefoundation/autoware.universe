cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocessor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)


###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


# ========== Compare Map Filter ==========
# -- Distance Based Compare Map Filter --
ament_auto_add_library(distance_based_compare_map_filter SHARED
  src/compare_map_filter/distance_based_compare_map_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(distance_based_compare_map_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(distance_based_compare_map_filter
  PLUGIN "pointcloud_preprocessor::DistanceBasedCompareMapFilterComponent"
  EXECUTABLE distance_based_compare_map_filter_node)

# -- Voxel Based Approximate Compare Map Filter --
ament_auto_add_library(voxel_based_approximate_compare_map_filter SHARED
  src/compare_map_filter/voxel_based_approximate_compare_map_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(voxel_based_approximate_compare_map_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(voxel_based_approximate_compare_map_filter
  PLUGIN "pointcloud_preprocessor::VoxelBasedApproximateCompareMapFilterComponent"
  EXECUTABLE voxel_based_approximate_compare_map_filter_node)

# -- Voxel Based Compare Map Filter --
ament_auto_add_library(voxel_based_compare_map_filter SHARED
  src/compare_map_filter/voxel_based_compare_map_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(voxel_based_compare_map_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(voxel_based_compare_map_filter
  PLUGIN "pointcloud_preprocessor::VoxelBasedCompareMapFilterComponent"
  EXECUTABLE voxel_based_compare_map_filter_node)

# -- Voxel Distance Based Compare Map Filter --
ament_auto_add_library(voxel_distance_based_compare_map_filter SHARED
  src/compare_map_filter/voxel_distance_based_compare_map_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(voxel_distance_based_compare_map_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(voxel_distance_based_compare_map_filter
  PLUGIN "pointcloud_preprocessor::VoxelDistanceBasedCompareMapFilterComponent"
  EXECUTABLE voxel_distance_based_compare_map_filter_node)


# ========== Concatenate data ==========
ament_auto_add_library(concatenate_data SHARED
  src/concatenate_data/concatenate_data_nodelet.cpp
  )
target_link_libraries(concatenate_data ${Boost_LIBRARIES})
rclcpp_components_register_node(concatenate_data
  PLUGIN "pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent"
  EXECUTABLE concatenate_data_node)


# ========== CropBox ==========
ament_auto_add_library(crop_box_filter SHARED
  src/crop_box_filter/crop_box_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(crop_box_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(crop_box_filter
  PLUGIN "pointcloud_preprocessor::CropBoxFilterComponent"
  EXECUTABLE crop_box_filter_node)


# ========== Down Sampler Filter ==========
ament_auto_add_library(downsample_filter SHARED
  src/downsample_filter/voxel_grid_downsample_filter_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(downsample_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(downsample_filter
  PLUGIN "pointcloud_preprocessor::VoxelGridDownsampleFilterComponent"
  EXECUTABLE downsample_filter_node)


# ========== Ray Ground Filter ==========
ament_auto_add_library(ray_ground_filter SHARED
  src/ground_filter/ray_ground_filter_nodelet.cpp
  src/filter.cpp)
target_link_libraries(ray_ground_filter ${OpenCV_LIBS})
if (OPENMP_FOUND)
  set_target_properties(ray_ground_filter PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
    )
endif ()
rclcpp_components_register_node(ray_ground_filter
  PLUGIN "pointcloud_preprocessor::RayGroundFilterComponent"
  EXECUTABLE ray_ground_filter_node)


# ========== Outlier Filter ==========
# -- Ring Outlier Filter --
ament_auto_add_library(ring_outlier_filter SHARED
  src/outlier_filter/ring_outlier_filter_nodelet.cpp
  src/filter.cpp)
target_link_libraries(ring_outlier_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(ring_outlier_filter
  PLUGIN "pointcloud_preprocessor::RingOutlierFilterComponent"
  EXECUTABLE ring_outlier_filter_node)

# -- Voxel Grid Outlier Filter --
ament_auto_add_library(voxel_grid_outlier_filter SHARED
  src/outlier_filter/voxel_grid_outlier_filter_nodelet.cpp
  src/filter.cpp)
target_link_libraries(voxel_grid_outlier_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(voxel_grid_outlier_filter
  PLUGIN "pointcloud_preprocessor::VoxelGridOutlierFilterComponent"
  EXECUTABLE voxel_grid_outlier_filter_node)


# ========== Passthrough Filter ==========
# -- Passthrough Filter --
ament_auto_add_library(passthrough_filter SHARED
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/filter.cpp)
target_link_libraries(passthrough_filter ${Boost_LIBRARIES})
rclcpp_components_register_node(passthrough_filter
  PLUGIN "pointcloud_preprocessor::PassThroughFilterComponent"
  EXECUTABLE passthrough_filter_node)

# -- Passthrough Filter Uint16 --
ament_auto_add_library(passthrough_filter_uint16 SHARED
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/filter.cpp)
target_link_libraries(passthrough_filter_uint16 ${Boost_LIBRARIES})
rclcpp_components_register_node(passthrough_filter_uint16
  PLUGIN "pointcloud_preprocessor::PassThroughFilterUInt16Component"
  EXECUTABLE passthrough_filter_uint16_node)


# ========== Pointcloud Accumulator Filter ==========
ament_auto_add_library(pointcloud_accumulator SHARED
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/filter.cpp
  )
target_link_libraries(pointcloud_accumulator ${Boost_LIBRARIES})
rclcpp_components_register_node(pointcloud_accumulator
  PLUGIN "pointcloud_preprocessor::PointcloudAccumulatorComponent"
  EXECUTABLE pointcloud_accumulator_node)




#############
## Install ##
#############

ament_auto_package(INSTALL_TO_SHARE
  launch
)
