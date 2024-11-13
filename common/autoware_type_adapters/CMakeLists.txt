# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# cspell:ignore NVTX

cmake_minimum_required(VERSION 3.5)

project(autoware_type_adapters)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # cspell:ignore-next-line
  add_compile_options(-Wall -Wextra -Wpedantic -g -funwind-tables)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(CUDA REQUIRED)

# Enable NVTX markers for improved profiling
add_definitions(-DUSE_NVTX)
link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
link_libraries("nvToolsExt")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Type Adapter
ament_auto_add_library(autoware_type_adapters SHARED
  src/image_container.cpp
  src/compressed_image_container.cpp
  src/cuda_stream_wrapper.cpp
)

target_include_directories(autoware_type_adapters PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(autoware_type_adapters
  ${CUDA_nvToolsExt_LIBRARY}
  ${CUDA_LIBRARIES}
)
ament_target_dependencies(autoware_type_adapters
  rclcpp
  sensor_msgs
)

# Add headers
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories("include")

install(TARGETS
  autoware_type_adapters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
