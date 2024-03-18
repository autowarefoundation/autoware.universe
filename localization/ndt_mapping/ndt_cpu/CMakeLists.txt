cmake_minimum_required(VERSION 2.8.3)
project(ndt_cpu)

find_package(PCL REQUIRED)

find_package(Eigen3 QUIET)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(
  "include"
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(srcs
  src/NormalDistributionsTransform.cpp
  src/Registration.cpp
  src/VoxelGrid.cpp
  src/Octree.cpp
)

set(incs
  include/ndt_cpu/debug.h
  include/ndt_cpu/NormalDistributionsTransform.h
  include/ndt_cpu/Registration.h
  include/ndt_cpu/SymmetricEigenSolver.h
  include/ndt_cpu/VoxelGrid.h
  include/ndt_cpu/Octree.h
)

if(NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Release"))
  message(WARNING "Not building for release, performance will be slow")

  message(WARNING
  "Adding 'EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT' macro to prevent ndt_matching's runtime error in debug mode.
    The bug reasons and solutions are written in http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html .
    This workaround was discussed on https://gitlab.com/autowarefoundation/autoware.ai/core_perception/merge_requests/57 .")
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()

add_library(ndt_cpu ${incs} ${srcs})

# ament_target_dependencies(ndt_cpu rclcpp std_msgs)

target_link_libraries(ndt_cpu
  ${PCL_LIBRARIES}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

# install(TARGETS ndt_cpu
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
