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

#ifndef AUTOWARE__PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_
#define AUTOWARE__PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_

#include <autoware_utils/geometry/geometry.hpp>

namespace autoware::path_smoother
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = autoware_utils::get_point(t1);
  const auto p2 = autoware_utils::get_point(t2);

  constexpr double epsilon = 1e-6;
  return (std::abs(p1.x - p2.x) <= epsilon && std::abs(p1.y - p2.y) <= epsilon);
}
}  // namespace geometry_utils
}  // namespace autoware::path_smoother
#endif  // AUTOWARE__PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_
