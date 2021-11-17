# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(point_type_adapter)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(POINT_TYPE_ADAPTER_NODE_SRC
  src/point_type_adapter_node.cpp)

set(POINT_TYPE_ADAPTER_NODE_HEADERS
  include/point_type_adapter/point_type_adapter_node.hpp
  include/point_type_adapter/visibility_control.hpp)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${POINT_TYPE_ADAPTER_NODE_SRC}
  ${POINT_TYPE_ADAPTER_NODE_HEADERS})
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::tools::point_type_adapter::PointTypeAdapterNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_point_type_adapter.cpp)
  set(TEST_POINT_TYPE_ADAPTER_EXE test_point_type_adapter)
  ament_add_gtest(${TEST_POINT_TYPE_ADAPTER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_POINT_TYPE_ADAPTER_EXE})
  target_link_libraries(${TEST_POINT_TYPE_ADAPTER_EXE} ${PROJECT_NAME}_node)
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch)
