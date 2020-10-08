cmake_minimum_required(VERSION 3.0.2)
project(shape_estimation)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  pcl_ros
  roscpp
  tf2
  tf2_geometry_msgs
  rostest
  rosunit
  eigen_conversions
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_perception_msgs
    pcl_ros
    roscpp
    tf2
    tf2_geometry_msgs
    eigen_conversions
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(shape_estimation
  src/main.cpp
  src/node.cpp
  src/shape_estimator.cpp
  src/model/normal/bounding_box.cpp
  src/model/normal/convex_hull.cpp
  src/model/normal/cylinder.cpp
  src/model/yaw_fixed/bounding_box.cpp
  src/filter/car_filter.cpp
  src/filter/bus_filter.cpp
  src/filter/truck_filter.cpp
  src/filter/no_filter.cpp
  src/corrector/normal/car_corrector.cpp
  src/corrector/normal/bus_corrector.cpp
  src/corrector/normal/truck_corrector.cpp
  src/corrector/normal/no_corrector.cpp
  src/corrector/yaw_fixed/car_corrector.cpp
  src/corrector/yaw_fixed/bus_corrector.cpp
  src/corrector/yaw_fixed/truck_corrector.cpp
  src/corrector/yaw_fixed/no_corrector.cpp
)

add_dependencies(shape_estimation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(shape_estimation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
install(TARGETS shape_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
