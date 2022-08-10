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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_

#include "apparent_safe_velocity_limiter/obstacles.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"

#include <grid_map_core/GridMap.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace apparent_safe_velocity_limiter
{

/// @brief mask gridmap cells that are inside the given polygons
/// @param[in, out] grid_map the grid map to modify
/// @param[in] polygons the polygons to mask from the grid map
void maskPolygons(grid_map::GridMap & grid_map, const ObstacleMasks & masks);

/// @brief apply a threshold to the grid map
/// @param[in, out] grid_map the grid map to modify
/// @param[in] threshold cells above this value are set to the max value, the other are set to 0
void threshold(grid_map::GridMap & grid_map, const float threshold);

grid_map::GridMap convertToGridMap(const OccupancyGrid & occupancy_grid);

/// @brief extract obstacles from an occupancy grid
/// @param[in] occupancy_grid input occupancy grid
/// @param[in] occupied_threshold threshold to use for identifying obstacles in the occupancy grid
/// @return extracted obstacle linestrings
multilinestring_t extractObstacles(
  const grid_map::GridMap & grid_map, const OccupancyGrid & occupancy_grid);
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__OCCUPANCY_GRID_UTILS_HPP_
