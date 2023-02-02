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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__UTILS__GEOMETRY_UTILS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__UTILS__GEOMETRY_UTILS_HPP_

#include "eigen3/Eigen/Core"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/type_alias.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const obstacle_avoidance_planner::ReferencePoint & p);

template <>
geometry_msgs::msg::Pose getPose(const obstacle_avoidance_planner::ReferencePoint & p);
}  // namespace tier4_autoware_utils

namespace obstacle_avoidance_planner
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = tier4_autoware_utils::getPoint(t1);
  const auto p2 = tier4_autoware_utils::getPoint(t2);

  constexpr double epsilon = 1e-6;
  if (epsilon < std::abs(p1.x - p2.x) || epsilon < std::abs(p1.y - p2.y)) {
    return false;
  }
  return true;
}

bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const vehicle_info_util::VehicleInfo & vehicle_info);

template <typename T>
std::vector<double> calcCurvature(const T & points, const size_t num_sampling_points)
{
  std::vector<double> res(points.size());
  const size_t num_points = static_cast<int>(points.size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  size_t max_smoothing_num = static_cast<size_t>(std::floor(0.5 * (num_points - 1)));
  size_t L = std::min(num_sampling_points, max_smoothing_num);
  for (size_t i = L; i < num_points - L; ++i) {
    p1 = tier4_autoware_utils::getPoint(points.at(i - L));
    p2 = tier4_autoware_utils::getPoint(points.at(i));
    p3 = tier4_autoware_utils::getPoint(points.at(i + L));
    double den = std::max(
      tier4_autoware_utils::calcDistance2d(p1, p2) * tier4_autoware_utils::calcDistance2d(p2, p3) *
        tier4_autoware_utils::calcDistance2d(p3, p1),
      0.0001);
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    res.at(i) = curvature;
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, num_points); ++i) {
    res.at(i) = res.at(std::min(L, num_points - 1));
    res.at(num_points - i - 1) =
      res.at(std::max(static_cast<int>(num_points) - static_cast<int>(L) - 1, 0));
  }
  return res;
}
}  // namespace geometry_utils
}  // namespace obstacle_avoidance_planner
#endif  // OBSTACLE_AVOIDANCE_PLANNER__UTILS__GEOMETRY_UTILS_HPP_
