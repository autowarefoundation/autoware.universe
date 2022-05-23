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

#include "grid_map_utils/polygon_iterator.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <grid_map_core/GridMapMath.hpp>

#include <algorithm>
#include <functional>
#include <utility>

namespace grid_map_utils
{

std::vector<Edge> PolygonIterator::calculateSortedEdges(const grid_map::Polygon & polygon)
{
  std::vector<Edge> edges;
  edges.reserve(polygon.nVertices() / 2);
  const auto & vertices = polygon.getVertices();
  for (auto vertex = vertices.cbegin(); std::next(vertex) != vertices.cend(); ++vertex) {
    // order pair by decreasing x and ignore horizontal edges (when x is equal)
    if (vertex->x() > std::next(vertex)->x())
      edges.emplace_back(*vertex, *std::next(vertex));
    else if (vertex->x() < std::next(vertex)->x())
      edges.emplace_back(*std::next(vertex), *vertex);
  }
  std::sort(edges.begin(), edges.end());
  edges.shrink_to_fit();
  return edges;
}

std::vector<std::list<double>> PolygonIterator::calculateIntersectionsPerLine(
  const std::vector<Edge> & edges, const std::pair<int, int> from_to_row,
  const grid_map::Position & origin, const grid_map::GridMap & grid_map)
{
  const auto from_row = from_to_row.first;
  const auto to_row = from_to_row.second;
  // calculate for each line the y value intersecting with the polygon in decreasing order
  std::vector<std::list<double>> y_intersections_per_line;
  y_intersections_per_line.reserve(to_row - from_row + 1);
  for (auto row = from_row; row <= to_row; ++row) {
    std::list<double> y_intersections;
    for (const auto & edge : edges) {
      const auto line_x = origin.x() - grid_map.getResolution() * row;
      // special case when exactly touching a vertex: only count edge for its lowest x
      // up-down edge (\/) case: count the vertex twice
      // down-down edge case: count the vertex only once
      if (edge.second.x() == line_x) {
        y_intersections.push_back(edge.second.y());
      } else if (edge.first.x() >= line_x && edge.second.x() < line_x) {
        const auto diff = edge.first - edge.second;
        const auto y = edge.second.y() + (line_x - edge.second.x()) * diff.y() / diff.x();
        y_intersections.push_back(y);
      } else if (edge.first.x() < line_x) {  // edge below the line
        break;
      }
    }
    y_intersections.sort(std::greater());
    // remove pairs outside of map
    auto iter = y_intersections.begin();
    while (iter != y_intersections.end() && std::next(iter) != y_intersections.end() &&
           *iter >= origin.y() && *std::next(iter) >= origin.y()) {
      iter = y_intersections.erase(iter);
      iter = y_intersections.erase(iter);
    }
    iter = std::lower_bound(
      y_intersections.begin(), y_intersections.end(),
      origin.y() - (grid_map.getSize()(1) - 1) * grid_map.getResolution(), std::greater());
    while (iter != y_intersections.end() && std::next(iter) != y_intersections.end()) {
      iter = y_intersections.erase(iter);
      iter = y_intersections.erase(iter);
    }
    y_intersections_per_line.push_back(y_intersections);
  }
  return y_intersections_per_line;
}

std::pair<int, int> PolygonIterator::calculateRowRange(
  const std::vector<Edge> & edges, const grid_map::Position & origin,
  const grid_map::GridMap & grid_map)
{
  const auto min_vertex_x = edges.back().second.x();
  const auto max_vertex_x = edges.front().first.x();

  const auto dist_min_to_origin = origin.x() - min_vertex_x + grid_map.getResolution();
  const auto dist_max_to_origin = origin.x() - max_vertex_x + grid_map.getResolution();
  const auto min_row = std::clamp(
    static_cast<int>(dist_max_to_origin / grid_map.getResolution()), 0, grid_map.getSize()(0) - 1);
  const auto max_row = std::clamp(
    static_cast<int>(dist_min_to_origin / grid_map.getResolution()), 0, grid_map.getSize()(0) - 1);

  return {min_row, max_row};
}

PolygonIterator::PolygonIterator(
  const grid_map::GridMap & grid_map, const grid_map::Polygon & polygon)
{
  auto poly = polygon;
  if (poly.nVertices() < 3) return;
  // repeat the first vertex to get the last edge [last vertex, first vertex]
  if (poly.getVertex(0) != poly.getVertex(poly.nVertices() - 1)) poly.addVertex(poly.getVertex(0));

  const auto resolution = grid_map.getResolution();
  const auto & map_size = grid_map.getSize();
  const auto & start_idx = grid_map.getStartIndex();
  const auto origin = [&]() {
    grid_map::Position origin;
    grid_map.getPosition(start_idx, origin);
    return origin;
  }();
  // We make line scan left -> right / up -> down *in the index frame* (idx[0,0] is pos[up, left]).
  // In the position frame, this corresponds to high -> low Y values and high -> low X values.
  const std::vector<Edge> edges = calculateSortedEdges(poly);
  if (edges.empty()) return;
  const auto from_to_row = calculateRowRange(edges, origin, grid_map);
  const auto y_intersections_per_line =
    calculateIntersectionsPerLine(edges, from_to_row, origin, grid_map);

  // calculate map indexes between pairs of intersections Y values on each X line
  polygon_indexes_.reserve(
    y_intersections_per_line.size() * static_cast<size_t>(grid_map.getSize()(1)));
  const auto from_row = start_idx(0) + from_to_row.first;
  for (size_t i = 0; i < y_intersections_per_line.size(); ++i) {
    const auto & y_intersections = y_intersections_per_line[i];
    auto row = static_cast<int>(from_row + i);
    grid_map::wrapIndexToRange(row, map_size(0));
    for (auto y_iter = y_intersections.cbegin(); y_iter != y_intersections.cend(); ++y_iter) {
      const auto dist_from_origin = origin.y() - *y_iter + resolution;
      const auto from_col =
        std::clamp(static_cast<int>(dist_from_origin / resolution), 0, map_size(1) - 1);

      ++y_iter;
      const auto dist_to_origin = origin.y() - *y_iter;
      const auto to_col =
        std::clamp(static_cast<int>(dist_to_origin / resolution), 0, map_size(1) - 1);
      for (auto col = from_col; col <= to_col; ++col) {
        auto wrapped_col = start_idx(1) + col;
        grid_map::wrapIndexToRange(wrapped_col, map_size(1));
        polygon_indexes_.emplace_back(row, wrapped_col);
      }
    }
  }
}

bool PolygonIterator::operator!=(const PolygonIterator & other) const
{
  return current_index != other.current_index;
}

const grid_map::Index & PolygonIterator::operator*() const
{
  return polygon_indexes_[current_index];
}

PolygonIterator & PolygonIterator::operator++()
{
  ++current_index;
  return *this;
}

[[nodiscard]] bool PolygonIterator::isPastEnd() const
{
  return current_index >= polygon_indexes_.size();
}
}  // namespace grid_map_utils
