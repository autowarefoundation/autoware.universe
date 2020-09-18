cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_preprocessor)

add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/CompareMapFilter.cfg
  cfg/VoxelGridOutlierFilter.cfg
  cfg/RingOutlierFilter.cfg
  cfg/VoxelGridDownsampleFilter.cfg
  cfg/PointcloudAccumulator.cfg
  cfg/PassThroughFilter.cfg
  cfg/PassThroughFilterUInt16.cfg
  cfg/RayGroundFilter.cfg
  cfg/CropBoxFilter.cfg
)

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Boost REQUIRED)
find_package (OpenMP)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_accumulator
  CATKIN_DEPENDS nodelet pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS Boost EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# Compare Map Filter
add_library(
  compare_map_filter
  src/compare_map_filter/distance_based_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_based_approximate_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_distance_based_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_based_compare_map_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(compare_map_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  distance_based_compare_map_filter_node
  src/compare_map_filter/distance_based_compare_map_filter_node.cpp)
target_link_libraries(distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(distance_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_based_approximate_compare_map_filter_node
  src/compare_map_filter/voxel_based_approximate_compare_map_filter_node.cpp
  )
target_link_libraries(voxel_based_approximate_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(voxel_based_approximate_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_distance_based_compare_map_filter_node
  src/compare_map_filter/voxel_distance_based_compare_map_filter_node.cpp
  )
target_link_libraries(voxel_distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(voxel_distance_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_based_compare_map_filter_node
  src/compare_map_filter/voxel_based_compare_map_filter_node.cpp)
target_link_libraries(voxel_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(voxel_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Outlier Filter
add_library(outlier_filter
            src/outlier_filter/voxel_grid_outlier_filter_nodelet.cpp
            src/outlier_filter/ring_outlier_filter_nodelet.cpp
            src/filter.cpp # copy from pcl_ros https://github.com/ros-
                           # perception/perception_pcl/issues/9
            )
target_link_libraries(outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(outlier_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(voxel_grid_outlier_filter_node
               src/outlier_filter/voxel_grid_outlier_filter_node.cpp)
target_link_libraries(voxel_grid_outlier_filter_node
                      outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(voxel_grid_outlier_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


add_executable(ring_outlier_filter_node
               src/outlier_filter/ring_outlier_filter_node.cpp)
target_link_libraries(ring_outlier_filter_node
                      outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(ring_outlier_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Downsample Filter
add_library(downsample_filter
            src/downsample_filter/voxel_grid_downsample_filter_nodelet.cpp
            src/filter.cpp # copy from pcl_ros https://github.com/ros-
                           # perception/perception_pcl/issues/9
            )
target_link_libraries(downsample_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(downsample_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(voxel_grid_downsample_filter_node
               src/downsample_filter/voxel_grid_downsample_filter_node.cpp)
target_link_libraries(voxel_grid_downsample_filter_node
                      downsample_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(voxel_grid_downsample_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Pointcloud Accumulator
add_library(pointcloud_accumulator
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(pointcloud_accumulator
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(pointcloud_accumulator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(pointcloud_accumulator_node
  src/pointcloud_accumulator/pointcloud_accumulator_node.cpp)
target_link_libraries(pointcloud_accumulator_node
                      pointcloud_accumulator
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(pointcloud_accumulator_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# PassThrough
add_library(
  passthrough_filter
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/passthrough_filter/passthrough_uint16.cpp
  src/passthrough_filter/passthrough_filter_uint16_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(passthrough_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(passthrough_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  passthrough_filter_node
  src/passthrough_filter/passthrough_filter_node.cpp)
target_link_libraries(passthrough_filter_node
                      passthrough_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(passthrough_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# PassThroughUInt16
add_executable(
  passthrough_filter_uint16_node
  src/passthrough_filter/passthrough_filter_uint16_node.cpp)
target_link_libraries(passthrough_filter_uint16_node
                      passthrough_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(passthrough_filter_uint16_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Ray Ground Filter
add_library(ground_filter
  src/ground_filter/ray_ground_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
if (OPENMP_FOUND)
  set_target_properties(ground_filter PROPERTIES
     COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
     LINK_FLAGS ${OpenMP_CXX_FLAGS}
     )
endif ()
target_include_directories(ground_filter PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ground_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(ground_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(ray_ground_filter_node
  src/ground_filter/ray_ground_filter_node.cpp)
target_link_libraries(ray_ground_filter_node
   ground_filter
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
add_dependencies(ray_ground_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# CropBox
add_library(
  crop_box_filter
  src/crop_box_filter/crop_box_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(crop_box_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(crop_box_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  crop_box_filter_node
  src/crop_box_filter/crop_box_filter_node.cpp)
target_link_libraries(crop_box_filter_node
                      crop_box_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(crop_box_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Concatenate data
add_library(
  concatenate_data
  src/concatenate_data/concatenate_data_nodelet.cpp
  )
target_link_libraries(concatenate_data
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(concatenate_data ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  concatenate_data_node
  src/concatenate_data/concatenate_data_node.cpp)
target_link_libraries(concatenate_data_node
                      concatenate_data
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})
add_dependencies(concatenate_data_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# Node
install(
  TARGETS
    crop_box_filter_node
    voxel_grid_downsample_filter_node
    passthrough_filter_uint16_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Nodelet
install(
  TARGETS
    compare_map_filter
    concatenate_data
    crop_box_filter
    concatenate_data
    downsample_filter
    ground_filter
    outlier_filter
    passthrough_filter
    pointcloud_accumulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
