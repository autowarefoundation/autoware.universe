# Copyright 2021 The Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

cmake_minimum_required(VERSION 3.5)
project(costmap_generator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(COSTMAP_GENERATOR_LIB costmap_generator)

ament_auto_add_library(${COSTMAP_GENERATOR_LIB} SHARED
  src/costmap_generator/costmap_generator.cpp
  src/costmap_generator/object_map_utils.cpp
)
autoware_set_compile_options(${COSTMAP_GENERATOR_LIB})

# skipping Werrors from external `grid_map_cv` package
target_compile_options(${COSTMAP_GENERATOR_LIB} PRIVATE -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion -Wno-double-conversion -Wno-double-promotion)

# if(BUILD_TESTING)
#   # run linters
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_auto_package(INSTALL_TO_SHARE)
