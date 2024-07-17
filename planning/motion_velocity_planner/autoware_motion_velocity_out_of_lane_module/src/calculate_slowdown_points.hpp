// Copyright 2024 TIER IV, Inc.
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

#ifndef CALCULATE_SLOWDOWN_POINTS_HPP_
#define CALCULATE_SLOWDOWN_POINTS_HPP_

#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <optional>

namespace autoware::motion_velocity_planner::out_of_lane
{
/// @brief calculate the last pose along the trajectory where ego does not overlap the lane to avoid
/// @param [in] ego_data ego data
/// @param [in] decision the input decision (i.e., which lane to avoid and at what speed)
/// @param [in] footprint the ego footprint
/// @param [in] params parameters
/// @return the last pose that is not out of lane (if found)
std::optional<geometry_msgs::msg::Pose> calculate_last_in_lane_pose(
  const EgoData & ego_data, const OutOfLanePoint & out_of_lane_point,
  const autoware::universe_utils::Polygon2d & footprint, const PlannerParam & params);

/// @brief calculate the slowdown point to insert in the trajectory
/// @param ego_data ego data (trajectory, velocity, etc)
/// @param decisions decision (before which point to stop, what lane to avoid entering, etc)
/// @param params parameters
/// @return optional slowdown point to insert in the trajectory
std::optional<geometry_msgs::msg::Pose> calculate_slowdown_point(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data, PlannerParam params);
}  // namespace autoware::motion_velocity_planner::out_of_lane
#endif  // CALCULATE_SLOWDOWN_POINTS_HPP_
