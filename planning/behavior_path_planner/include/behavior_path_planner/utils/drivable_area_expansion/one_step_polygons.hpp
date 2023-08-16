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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__ONE_STEP_POLYGONS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__ONE_STEP_POLYGONS_HPP_

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <boost/geometry.hpp>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace polygon_utils
{
namespace bg = boost::geometry;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

std::vector<Polygon2d> createOneStepPolygons(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double lat_margin = 0.0);
}  // namespace polygon_utils

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__ONE_STEP_POLYGONS_HPP_
