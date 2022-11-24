// Copyright 2022 TIER IV, Inc.
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

#ifndef DRIVABLE_AREA_EXPANDER__GRID_UTILS_HPP_
#define DRIVABLE_AREA_EXPANDER__GRID_UTILS_HPP_

#include "drivable_area_expander/types.hpp"

#include <grid_map_core/GridMap.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace drivable_area_expander
{

/// @brief mask gridmap cells that are inside the given polygons
/// @param[in, out] grid_map the grid map to modify
/// @param[in] polygons the polygons to mask from the grid map
void maskPolygon(grid_map::GridMap & grid_map, const polygon_t & polygon);

/// @brief convert an OccupancyGrid to a GridMap object
/// @param[in] occupancy_grid grid to convert
/// @return converted GridMap
grid_map::GridMap convertToGridMap(const OccupancyGrid & occupancy_grid);

/// @brief convert a GridMap to an OccupancyGrid
/// @param[in] grid_map grid to convert
/// @return converted OccupancyGrid
OccupancyGrid convertToOccupancyGrid(const grid_map::GridMap & grid_map);
}  // namespace drivable_area_expander

#endif  // DRIVABLE_AREA_EXPANDER__GRID_UTILS_HPP_
