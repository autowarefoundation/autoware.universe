// Copyright 2023-2024 TIER IV, Inc.
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

#ifndef COLLISION_HPP_
#define COLLISION_HPP_

#include "types.hpp"

#include <optional>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{
/// @brief find the collision point closest to ego along an object footprint
/// @param [in] ego_data ego data including its path and footprints used for finding a collision
/// @param [in] object_pose pose of the dynamic object
/// @param [in] object_footprint footprint of the object used for finding a collision
/// @return the collision point closest to ego (if any)
std::optional<geometry_msgs::msg::Point> find_closest_collision_point(
  const EgoData & ego_data, const autoware_perception_msgs::msg::PredictedObject & object,
  const tier4_autoware_utils::MultiPolygon2d & object_footprints, const PlannerParam & params);

/// @brief find the earliest collision along the ego path
/// @param [in] ego_data ego data including its path and footprint
/// @param [in] objects obstacles
/// @param [in] obstacle_forward_footprints obstacle footprints to check for collisions
/// @return the point of earliest collision along the ego path
std::vector<Collision> find_collisions(
  const EgoData & ego_data,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const std::vector<tier4_autoware_utils::MultiPolygon2d> & object_forward_footprints,
  const PlannerParam & params);

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // COLLISION_HPP_
