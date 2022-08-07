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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__DISTANCE_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__DISTANCE_HPP_

#include "apparent_safe_velocity_limiter/obstacles.hpp"
#include "apparent_safe_velocity_limiter/parameters.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"

#include <geometry_msgs/msg/vector3.hpp>

#include <optional>
#include <vector>

namespace apparent_safe_velocity_limiter
{
/// @brief calculate the closest distance to a collision
/// @param [in] projection forward projection line
/// @param [in] footprint footprint of the projection
/// @param [in] obstacles set of obstacles to check for collision
/// @param [in] params projection parameters
/// @param [in] max_obstacle_distance optional maximum distance for obstacles to be considered
/// @return distance to the closest collision if any
std::optional<double> distanceToClosestCollision(
  const linestring_t & projection, const polygon_t & footprint, const Obstacles & obstacles,
  const ProjectionParameters & params, const std::optional<double> max_obstacle_distance);

/// @brief calculate the closest distance along a circle to a given target point
/// @param [in] origin starting point
/// @param [in] heading heading used to calculate the tangent to the circle at the origin
/// @param [in] target target point
/// @return distance from origin to target along a circle
double arcDistance(const point_t & origin, const double heading, const point_t & target);

}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__DISTANCE_HPP_
