cmake_minimum_required(VERSION 3.5)

project(vector_map_visual_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# rclpy
find_package(rclpy REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# YAML
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# ceres
find_package(Ceres REQUIRED)

# ===================================================
ament_auto_package(INSTALL_TO_SHARE launch config)