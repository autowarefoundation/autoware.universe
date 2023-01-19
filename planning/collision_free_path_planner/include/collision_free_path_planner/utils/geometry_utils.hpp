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

#ifndef COLLISION_FREE_PATH_PLANNER__UTILS__GEOMETRY_UTILS_HPP_
#define COLLISION_FREE_PATH_PLANNER__UTILS__GEOMETRY_UTILS_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "eigen3/Eigen/Core"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct collision_free_path_planner::ReferencePoint;

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const collision_free_path_planner::ReferencePoint & p);

template <>
geometry_msgs::msg::Pose getPose(const collision_free_path_planner::ReferencePoint & p);
}  // namespace tier4_autoware_utils

namespace collision_free_path_planner
{
namespace geometry_utils
{
bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const vehicle_info_util::VehicleInfo & vehicle_info);
}  // namespace geometry_utils
}  // namespace collision_free_path_planner
#endif  // COLLISION_FREE_PATH_PLANNER__UTILS__GEOMETRY_UTILS_HPP_
