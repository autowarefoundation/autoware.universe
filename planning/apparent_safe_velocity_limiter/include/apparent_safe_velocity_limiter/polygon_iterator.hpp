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

#ifndef APPARENT_SAFE_VELOCITY_LIMITER__POLYGON_ITERATOR_HPP_
#define APPARENT_SAFE_VELOCITY_LIMITER__POLYGON_ITERATOR_HPP_

#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

#include <grid_map_core/GridMap.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace apparent_safe_velocity_limiter
{

/// @brief More efficient polygon iterator using scan line algorithm
class PolygonIterator
{
  grid_map::Polygon polygon_;
  std::vector<grid_map::Index> polygon_indexes_;
  size_t current_index = 0;

public:
  PolygonIterator(const grid_map::GridMap & grid_map, const grid_map::Polygon & polygon)
  : polygon_(polygon)
  {
    if (polygon_.nVertices() < 3) return;
    // if needed repeat the first vertex to naturally get the last edge
    if (polygon_.getVertex(0) != polygon_.getVertex(polygon_.nVertices() - 1))
      polygon_.addVertex(polygon_.getVertex(0));

    const auto resolution = grid_map.getResolution();
    const auto & map_pose = grid_map.getPosition();
    const auto min_map_x = map_pose.x() - grid_map.getLength().x() / 2.0 + resolution / 2;
    const auto max_map_x = map_pose.x() + grid_map.getLength().x() / 2.0 - resolution / 2;
    const auto min_map_y = map_pose.y() - grid_map.getLength().y() / 2.0 + resolution / 2;
    const auto max_map_y = map_pose.y() + grid_map.getLength().y() / 2.0 - resolution / 2;

    // We make line scan from left to right / up to down *in the index frame*.
    // In the position frame, this corresponds to high to low Y values and high to low X values.
    // Thus, we make lines along the Y axis from highest to lowest X values.
    // This results in lexicographically ordered indexes.

    // get edges sorted from highest to lowest X values
    using Edge = std::pair<const grid_map::Position, const grid_map::Position>;
    struct Comp
    {
      bool operator()(const Edge & e1, const Edge & e2) const
      {
        return e1.first.x() > e2.first.x() ||
               (e1.first.x() == e2.first.x() && e1.second.x() > e2.second.x());
      }
    };

    tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stopwatch;
    double edges_d{};
    double inter_d{};
    double index_d{};

    stopwatch.tic("edges");
    std::multiset<Edge, Comp> edges;
    const auto & vertices = polygon_.getVertices();
    for (auto vertex = vertices.cbegin(); std::next(vertex) != vertices.cend(); ++vertex) {
      const auto edge = vertex->x() > std::next(vertex)->x() ?  // order pair by decreasing x
                          Edge(*vertex, *std::next(vertex))
                                                             : Edge(*std::next(vertex), *vertex);
      if (edge.first.x() != edge.second.x()) {  // ignore horizontal edges
        edges.insert(edge);
      }
    }
    edges_d += stopwatch.toc("edges");
    // get min/max x edge values
    const auto max_x = edges.cbegin()->first.x();
    const auto min_x = std::prev(edges.cend())->second.x();
    // get min/max x values truncated to grid cell centers
    const auto max_line_x = std::clamp(
      min_map_x + resolution * static_cast<int>((max_x - min_map_x) / resolution), min_map_x,
      max_map_x);
    const auto min_line_x = std::clamp(
      min_map_x + resolution * static_cast<int>((min_x - min_map_x) / resolution), min_map_x,
      max_map_x);
    // calculate for each line the y value intersecting with the polygon in decreasing order
    stopwatch.tic("inter");
    std::vector<std::multiset<double, std::greater<>>> y_intersections_per_line;
    const auto nb_x_lines = static_cast<size_t>((max_line_x - min_line_x) / resolution) + 1;
    y_intersections_per_line.reserve(nb_x_lines);
    const auto epsilon = resolution / 2.0;
    for (auto line_x = max_line_x; line_x >= min_line_x - epsilon; line_x -= resolution) {
      std::multiset<double, std::greater<>> y_intersections;
      for (const auto & edge : edges) {
        if (edge.first.x() < line_x) {  // edge below the line
          break;
        }
        // special case when exactly touching a vertex: only count edge for its lowest x
        // up-down edge (\/) case: count the vertex twice
        // down-down edge case: count the vertex only once
        if (edge.second.x() == line_x) {
          y_intersections.insert(edge.second.y());
        } else if (edge.first.x() > line_x && edge.second.x() < line_x) {
          const auto diff = edge.first - edge.second;
          const auto y = edge.second.y() + (line_x - edge.second.x()) * diff.y() / diff.x();
          y_intersections.insert(y);
        }
      }
      y_intersections_per_line.push_back(y_intersections);
    }
    inter_d += stopwatch.toc("inter");
    // calculate map indexes between pairs of intersections Y values on each X line
    grid_map::Position pos;
    grid_map::Index idx;
    stopwatch.tic("index");
    polygon_indexes_.reserve(
      nb_x_lines * static_cast<size_t>((max_map_y - min_map_y) / resolution + 1));
    for (size_t i = 0; i < y_intersections_per_line.size(); ++i) {
      const auto y_intersections = y_intersections_per_line[i];
      pos.x() = max_line_x - static_cast<double>(i) * resolution;
      grid_map.getIndex(pos, idx);
      for (auto y_iter = y_intersections.cbegin(); y_iter != y_intersections.cend(); ++y_iter) {
        const auto from_y = std::clamp(*y_iter, min_map_y, max_map_y + resolution);
        const auto from_col =
          static_cast<int>(std::abs(max_map_y + resolution - from_y) / resolution);

        ++y_iter;
        const auto to_y = std::clamp(*y_iter, min_map_y, max_map_y);
        const auto to_col = static_cast<int>(std::abs(max_map_y - to_y) / resolution);
        // special case where both intersection points are outside of the map
        if (
          (from_y >= max_map_y && to_y >= max_map_y) || (from_y <= min_map_y && to_y <= min_map_y))
          continue;
        for (auto col = from_col; col <= to_col; ++col) {
          polygon_indexes_.emplace_back(idx(0), col);
        }
      }
    }
    index_d += stopwatch.toc("index");
    std::cout << "Edges: " << edges_d << std::endl;
    std::cout << "Inter: " << inter_d << std::endl;
    std::cout << "Index: " << index_d << std::endl;
  }

  bool operator!=(const PolygonIterator & other) const
  {
    return current_index != other.current_index;
  }

  const grid_map::Index & operator*() const { return polygon_indexes_[current_index]; }

  PolygonIterator & operator++()
  {
    ++current_index;
    return *this;
  }

  [[nodiscard]] bool isPastEnd() const { return current_index >= polygon_indexes_.size(); }
};
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__POLYGON_ITERATOR_HPP_
