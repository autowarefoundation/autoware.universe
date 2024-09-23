// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_
#define AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_

#include "autoware/interpolation/interpolation_utils.hpp"

#include <vector>

namespace autoware::interpolation
{
double lerp(const double src_val, const double dst_val, const double ratio);

std::vector<double> lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);

double lerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const double query_key);
}  // namespace autoware::interpolation

#endif  // AUTOWARE__INTERPOLATION__LINEAR_INTERPOLATION_HPP_
