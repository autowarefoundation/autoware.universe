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

#include "drivable_area_expander/grid_utils.hpp"

#include "drivable_area_expander/types.hpp"

#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>

#include "nav_msgs/msg/detail/occupancy_grid__struct.hpp"

namespace drivable_area_expander
{
void maskPolygon(grid_map::GridMap & grid_map, const polygon_t & polygon)
{
  grid_map::Polygon poly;
  for (const auto & p : polygon.outer()) poly.addVertex(p);

  auto & layer = grid_map["layer"];

  for (grid_map_utils::PolygonIterator iterator(grid_map, poly); !iterator.isPastEnd(); ++iterator)
    layer((*iterator)(0), (*iterator)(1)) = 0.0;
}

grid_map::GridMap convertToGridMap(const OccupancyGrid & occupancy_grid)
{
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  return grid_map;
}

OccupancyGrid convertToOccupancyGrid(const grid_map::GridMap & grid_map)
{
  OccupancyGrid occupancy_grid;
  // TODO(Maxime): get proper max grid value ?
  grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, "layer", 0, 255, occupancy_grid);
  return occupancy_grid;
}
}  // namespace drivable_area_expander
