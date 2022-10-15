// Copyright 2022 Tier IV, Inc.
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
#ifndef COLLISION_FREE_PATH_PLANNER__DEBUG_MARKER_HPP_
#define COLLISION_FREE_PATH_PLANNER__DEBUG_MARKER_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "collision_free_path_planner/type_alias.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/clock.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <memory>
#include <string>
#include <vector>

namespace collision_free_path_planner
{
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

MarkerArray getDebugMarker(
  const DebugData & debug_data,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const vehicle_info_util::VehicleInfo & vehicle_info, const bool is_showing_debug_detail);

MarkerArray getDebugWallMarker(
  DebugData & debug_data, const vehicle_info_util::VehicleInfo & vehicle_info,
  const rclcpp::Time & now);

OccupancyGrid getDebugCostmap(const cv::Mat & clearance_map, const OccupancyGrid & occupancy_grid);
}  // namespace collision_free_path_planner
#endif  // COLLISION_FREE_PATH_PLANNER__DEBUG_MARKER_HPP_
