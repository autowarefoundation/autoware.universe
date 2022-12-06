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
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <deque>

namespace
{
constexpr auto layer_name = "layer";
constexpr auto undrivable = 100;
constexpr auto drivable = 0;
}  // namespace

namespace drivable_area_expander
{
grid_map::GridMap expandGridMap(
  const grid_map::GridMap & base_map, const multipolygon_t & footprint,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const point_t & origin)
{
  grid_map::GridMap footprint_map = base_map;
  maskPolygons(footprint_map, footprint, drivable);
  maskPolygons(footprint_map, predicted_paths, undrivable);
  maskLines(footprint_map, uncrossable_lines, undrivable);
  maskUnconnected(footprint_map, origin);
  return footprint_map;
}

void maskPolygons(grid_map::GridMap & grid_map, const multipolygon_t & polygons, const float value)
{
  auto & layer = grid_map[layer_name];

  grid_map::Polygon poly;
  for (const auto & polygon : polygons) {
    poly.removeVertices();
    for (const auto & p : polygon.outer()) poly.addVertex(p);
    for (grid_map_utils::PolygonIterator iterator(grid_map, poly); !iterator.isPastEnd();
         ++iterator)
      layer((*iterator)(0), (*iterator)(1)) = value;
  }
}

void maskLines(
  grid_map::GridMap & grid_map, const multilinestring_t & linestrings, const float value)
{
  auto & layer = grid_map[layer_name];

  for (const auto & line : linestrings) {
    for (auto i = 0lu; i + 1 < line.size(); ++i) {
      const grid_map::Position start(line[i].x(), line[i].y());
      const grid_map::Position end(line[i + 1].x(), line[i + 1].y());
      if (grid_map.isInside(start) && grid_map.isInside(end)) {
        for (grid_map::LineIterator iter(grid_map, start, end); !iter.isPastEnd(); ++iter)
          layer((*iter).x(), (*iter).y()) = value;
      }
    }
  }
}

void maskUnconnected(grid_map::GridMap & grid_map, const point_t & origin)
{
  auto & layer = grid_map[layer_name];
  const auto original_layer = layer;
  grid_map.add(layer_name, undrivable);

  std::vector<bool> visited(grid_map.getSize().x() * grid_map.getSize().y(), false);
  const auto index_of = [&](const auto & idx) {
    return idx.x() * grid_map.getSize().y() + idx.y();
  };
  grid_map::Index origin_idx;
  grid_map.getIndex(grid_map::Position(origin.x(), origin.y()), origin_idx);
  std::deque<grid_map::Index> to_visit = {origin_idx};
  const auto add_valid_neighbors = [&](const auto & idx) {
    if (idx.x() - 1 > 0) to_visit.emplace_back(idx.x() - 1, idx.y());
    if (idx.y() - 1 > 0) to_visit.emplace_back(idx.x(), idx.y() - 1);
    if (idx.x() + 1 < grid_map.getSize().x()) to_visit.emplace_back(idx.x() + 1, idx.y());
    if (idx.y() + 1 < grid_map.getSize().y()) to_visit.emplace_back(idx.x(), idx.y() + 1);
  };
  while (!to_visit.empty()) {
    const auto idx = to_visit.back();
    to_visit.pop_back();
    if (!visited[index_of(idx)]) {
      visited[index_of(idx)] = true;
      if (original_layer(idx.x(), idx.y()) == drivable) {
        layer(idx.x(), idx.y()) = drivable;
        add_valid_neighbors(idx);
      }
    }
  }
}

grid_map::GridMap convertToGridMap(const OccupancyGrid & occupancy_grid)
{
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, layer_name, grid_map);
  return grid_map;
}

OccupancyGrid convertToOccupancyGrid(const grid_map::GridMap & grid_map)
{
  OccupancyGrid occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    grid_map, layer_name, drivable, undrivable, occupancy_grid);
  return occupancy_grid;
}
}  // namespace drivable_area_expander
