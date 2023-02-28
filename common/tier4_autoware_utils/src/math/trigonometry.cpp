// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tier4_autoware_utils/math/constants.hpp>
#include <tier4_autoware_utils/math/sin_table.hpp>
#include <tier4_autoware_utils/math/trigonometry.hpp>

#include <cmath>

namespace tier4_autoware_utils {

float sin(float radian) {
  float degree = radian * (180.f / tier4_autoware_utils::pi) * (SIN_TABLE_SIZE_MUL4 / 360.f);
  size_t idx = (static_cast<int>(std::round(degree)) % SIN_TABLE_SIZE_MUL4 + SIN_TABLE_SIZE_MUL4) % SIN_TABLE_SIZE_MUL4;

  int mul = 1;
  if (SIN_TABLE_SIZE <= idx && idx < 2 * SIN_TABLE_SIZE) {
    idx = 2 * SIN_TABLE_SIZE - idx;
  } else if (2 * SIN_TABLE_SIZE <= idx && idx < 3 * SIN_TABLE_SIZE) {
    mul = -1;
    idx = idx - 2 * SIN_TABLE_SIZE;
  } else if (3 * SIN_TABLE_SIZE <= idx && idx < 4 * SIN_TABLE_SIZE) {
    mul = -1;
    idx = 4 * SIN_TABLE_SIZE - idx;
  }

  return mul * SIN_TABLE[idx];
}

float cos(float radian) {
  return sin(radian + tier4_autoware_utils::pi / 2.f);
}

} // namespace tier4_autoware_utils
