// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_
#define SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <vector>

namespace behavior_velocity_planner
{
namespace out_of_lane
{
tier4_autoware_utils::Polygon2d make_base_footprint(
  const PlannerParam & p, const bool ignore_offset = false);

lanelet::BasicPolygon2d project_to_pose(
  const tier4_autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose);

/// @brief calculate the path footprints
/// @details the resulting polygon follows the format used by the lanelet library: clockwise order
/// and implicit closing edge
/// @param [in] path input path
/// @param [in] first_idx first path index to consider
/// @param [in] params parameters
/// @return polygon footprints for each path point starting from first_idx
std::vector<lanelet::BasicPolygon2d> calculate_path_footprints(
  const EgoData & ego_data, const PlannerParam & params);

lanelet::BasicPolygon2d calculate_current_ego_footprint(
  const EgoData & ego_data, const PlannerParam & params, const bool ignore_offset = false);

/// @brief calculate points along the path where we want ego to slowdown/stop
/// @param ego_data ego data (path, velocity, etc)
/// @param decisions decisions (before which point to stop, what lane to avoid entering, etc)
/// @param params parameters
/// @return precise points to insert in the path
std::vector<SlowdownToInsert> calculate_slowdown_points(
  const EgoData & ego_data, std::vector<Slowdown> & decisions, PlannerParam params);
}  // namespace out_of_lane
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_
