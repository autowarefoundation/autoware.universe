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

#ifndef AUTOWARE__PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_
#define AUTOWARE__PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/type_alias.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <Eigen/Core>

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware_utils
{
template <>
geometry_msgs::msg::Point get_point(const autoware::path_optimizer::ReferencePoint & p);

template <>
geometry_msgs::msg::Pose get_pose(const autoware::path_optimizer::ReferencePoint & p);
}  // namespace autoware_utils

namespace autoware::path_optimizer
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = autoware_utils::get_point(t1);
  const auto p2 = autoware_utils::get_point(t2);

  constexpr double epsilon = 1e-6;
  if (epsilon < std::abs(p1.x - p2.x) || epsilon < std::abs(p1.y - p2.y)) {
    return false;
  }
  return true;
}

bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const bool use_footprint_polygon_for_outside_drivable_area_check);
}  // namespace geometry_utils
}  // namespace autoware::path_optimizer
#endif  // AUTOWARE__PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_
