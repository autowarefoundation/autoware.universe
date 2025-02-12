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

#ifndef AUTOWARE_PATH_SAMPLER__UTILS__GEOMETRY_UTILS_HPP_
#define AUTOWARE_PATH_SAMPLER__UTILS__GEOMETRY_UTILS_HPP_

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware_path_sampler/common_structs.hpp"
#include "autoware_path_sampler/type_alias.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "eigen3/Eigen/Core"

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::path_sampler
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = autoware::universe_utils::getPoint(t1);
  const auto p2 = autoware::universe_utils::getPoint(t2);

  constexpr double epsilon = 1e-6;
  if (epsilon < std::abs(p1.x - p2.x) || epsilon < std::abs(p1.y - p2.y)) {
    return false;
  }
  return true;
}
}  // namespace geometry_utils
}  // namespace autoware::path_sampler
#endif  // AUTOWARE_PATH_SAMPLER__UTILS__GEOMETRY_UTILS_HPP_
