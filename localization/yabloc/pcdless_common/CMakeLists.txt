cmake_minimum_required(VERSION 3.5)
project(pcdless_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# ===================================================
# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PAHT_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

# ===================================================
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ublox_stamp.cpp
  src/pub_sub.cpp
  src/fix2mgrs.cpp
  src/camera_info_subscriber.cpp
  src/cv_decompress.cpp
  src/pose_conversions.cpp
  src/static_tf_subscriber.cpp
  src/color.cpp)
target_link_libraries(${PROJECT_NAME} Geographic ${PCL_LIBRARIES} Sophus::Sophus)
target_include_directories(
  ${PROJECT_NAME} PRIVATE
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

ament_export_dependencies(PCL Sophus)

# ===================================================
ament_auto_package()
