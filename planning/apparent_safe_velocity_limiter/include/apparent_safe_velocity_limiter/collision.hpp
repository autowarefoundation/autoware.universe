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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_HPP_

#include "apparent_safe_velocity_limiter/types.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <optional>

namespace apparent_safe_velocity_limiter
{
/// @brief calculate the closest distance to a collision
/// @param [in] projection forward projection line
/// @param [in] footprint footprint of the projection
/// @param [in] obstacles set of obstacles as linestrings
/// @return distance to the closest collision if any
std::optional<double> distanceToClosestCollision(
  const linestring_t & projection, const polygon_t & footprint,
  const multilinestring_t & obstacles);

// TODO(Maxime CLEMENT): move to occupancy_grid_utils
/// @brief create a polygon from an object represented by a pose and a size
/// @param [in] pose pose of the object
/// @param [in] dimensions dimensions of the object
/// @param [in] buffer buffer to add to the dimensions of the object
/// @return polygon of the object
polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer);

/// @brief create polygons from a set of predicted object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the objects
multipolygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__COLLISION_HPP_
