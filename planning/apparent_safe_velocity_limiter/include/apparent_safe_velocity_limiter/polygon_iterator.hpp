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
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include <grid_map_core/GridMap.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
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
    PolygonIterator(const grid_map::GridMap& grid_map, const grid_map::Polygon& polygon)
        : polygon_(polygon)
{
    constexpr auto debug = false;
    if(polygon_.nVertices() < 3) return;
    // if needed repeat the first vertex to naturally get the last edge
    if(polygon_.getVertex(0) != polygon_.getVertex(polygon_.nVertices()-1))
        polygon_.addVertex(polygon_.getVertex(0));
    if(debug) {
        std::cout << "Polygon:\n";
        for(const auto & v : polygon_.getVertices())
            std::cout << "\t" << v.transpose() << "\n";
    }

    const auto resolution = grid_map.getResolution();
    const auto min_map_x = grid_map.getPosition().x() - grid_map.getLength().x() / 2.0 + resolution / 2;
    const auto max_map_x = grid_map.getPosition().x() + grid_map.getLength().x() / 2.0 - resolution / 2;
    const auto min_map_y = grid_map.getPosition().y() - grid_map.getLength().y() / 2.0 + resolution / 2;
    const auto max_map_y = grid_map.getPosition().y() + grid_map.getLength().y() / 2.0 - resolution / 2;
    if(debug) {
        std::printf("min/max x = (%2.2f, %2.2f) | min/max y = (%2.2f, %2.2f)\n", min_map_x, max_map_x, min_map_y, max_map_y);
        std::cout << "Map:\n";
        for(grid_map::GridMapIterator iter(grid_map); !iter.isPastEnd(); ++iter) {
            grid_map::Position pos;
            grid_map.getPosition(*iter, pos);
            std::cout << "\t[" << (*iter).transpose() << ", " << pos.transpose() << "]\n";
        }
    }

    // We make line scan from left to right / up to down *in the index frame*.
    // In the position frame, this corresponds to high to low Y values and high to low X values.
    // Thus, we make lines along the Y axis from highest to lowest X values.
    // This results in lexicographically ordered indexes.

    // get edges sorted from highest to lowest X values
    using Edge = std::pair<const grid_map::Position, const grid_map::Position>;
    struct Comp {
        bool operator()(const Edge & e1, const Edge & e2) const {
            return e1.first.x() > e2.first.x() || (e1.first.x() == e2.first.x() && e1.second.x() > e2.second.x());
        }
    };
    std::multiset<Edge, Comp> edges;
    const auto & vertices = polygon_.getVertices();
    for(auto vertex = vertices.cbegin(); std::next(vertex) != vertices.cend(); ++vertex) {
        const auto edge = vertex->x() > std::next(vertex)->x() ? // order pair by decreasing x
            std::make_pair(*vertex, *std::next(vertex)) : std::make_pair(*std::next(vertex), *vertex);
        if(edge.first.x() != edge.second.x()) { // ignore horizontal edges
            if(debug)
                std::cout << "insert " << edge.first.transpose() << " + " << edge.second.transpose() << std::endl;
            edges.insert(edge);
        }
        else {
            if(debug)
                std::cout << "ignore " << edge.first.transpose() << " + " << edge.second.transpose() << std::endl;
        }
    }
    if(debug) {
        std::cout << "Edges: \n";
        for(const auto & edge : edges) {
            std::cout << "\t" << edge.first.transpose() << ", " << edge.second.transpose() << "\n";
        }
    }
    // get min/max x edge values
    const auto max_x = edges.cbegin()->first.x();
    const auto min_x = std::prev(edges.cend())->second.x();
    // get min/max x values truncated to grid cell centers
    const auto max_line_x = std::clamp(min_map_x + resolution * static_cast<int>((max_x - min_map_x) / resolution), min_map_x, max_map_x);
    const auto min_line_x = std::clamp(min_map_x + resolution * static_cast<int>((min_x - min_map_x) / resolution), min_map_x, max_map_x);
    if(debug) {
        std::cout << "*** min/max x  :" << min_x << ", " << max_x << std::endl;
        std::cout << "*** min/max line_x  :" << min_line_x << ", " << max_line_x << std::endl;
    }
    // calculate for each line the y value intersecting with the polygon in decreasing order
    std::vector<std::multiset<double, std::greater<>>> y_intersections_per_line;
    y_intersections_per_line.reserve((max_line_x - min_line_x) / resolution + 1);
    if(debug)
        std::printf("y_intersections_per_line (size %lu):\n", y_intersections_per_line.capacity());
    const auto epsilon = resolution / 2.0;
    for(auto line_x = max_line_x; line_x >= min_line_x - epsilon; line_x -= resolution) {
        std::multiset<double, std::greater<>> y_intersections;
        if(debug)
            std::cout << "\tline x =" << line_x << "\n";
        for(const auto & edge : edges) {
           if(edge.first.x() < line_x) {  // edge below the line
               break;
           } 
           // special case when exactly touching a vertex: only count edge for its lowest x
           // up-down edge (\/) case: count the vertex twice
           // down-down edge case: count the vertex only once
           if(edge.second.x() == line_x) {
               y_intersections.insert(edge.second.y());
           }
           else if(edge.first.x() > line_x && edge.second.x() < line_x) {
                const auto diff = edge.first - edge.second;
                const auto y = edge.second.y() + (line_x - edge.second.x()) * diff.y() / diff.x();
                y_intersections.insert(y);
                if(debug)
                    std::cout << "\t\t" << diff.transpose() << " -> " << y << std::endl;
           }
        }
        if(debug){
                std::cout << "\t\t";
            for(const auto & y: y_intersections) {
                std::cout << y << " ";
            }
            std::cout << std::endl;
        }
        y_intersections_per_line.push_back(y_intersections);
    }
    // calculate map indexes between pairs of intersections Y values on each X line
    grid_map::Position pos;
    grid_map::Index idx;
    pos.y() = min_map_y;  // arbitrary value TODO can be removed ?
    for(size_t i = 0; i < y_intersections_per_line.size(); ++i) {
        const auto y_intersections = y_intersections_per_line[i];
        pos.x() = max_line_x - static_cast<double>(i) * resolution;
        grid_map.getIndex(pos, idx);
        if(debug)
            std::cout << "****** i = " << i << std::endl;
        for(auto y_iter = y_intersections.cbegin(); y_iter != y_intersections.cend(); ++y_iter) {
            const auto from_y = std::clamp(*y_iter, min_map_y, max_map_y + resolution);
            const auto from_col = static_cast<int>(std::abs(max_map_y + resolution - from_y) / resolution);

            if(debug) {
                std::cout << "*** RAW from y: " << *y_iter << std::endl;
                std::cout << "*** CLAMPED from y: " << from_y << std::endl;
                std::cout << "*** from_col: " << from_col << std::endl;
            }
            ++y_iter;
            const auto to_y = std::clamp(*y_iter, min_map_y, max_map_y);
            const auto to_col = static_cast<int>(std::abs(max_map_y - to_y) / resolution);
            if(debug) {
                std::cout << "*** RAW to y: " << *y_iter << std::endl;
                std::cout << "*** CLAMPED to y: " << to_y << std::endl;
                std::cout << "*** to_col: " << to_col << std::endl;
            }
            // special case where both intersection points are outside of the map
            if((from_y >= max_map_y && to_y >= max_map_y) || (from_y <= min_map_y && to_y <= min_map_y)) continue;
            for(idx(1) = from_col; idx(1) <= to_col; ++idx(1)) {
                if(debug) {
                    grid_map::Position tmp;
                    grid_map.getPosition(idx, tmp);
                    std::cout << "\t Position: " << tmp.transpose() << std::endl;
                    std::cout << "\t Index: " << idx.transpose() << std::endl;
                }
                polygon_indexes_.push_back(idx);
            }
        }
    }
}

bool operator !=(const PolygonIterator& other) const
{
  return current_index != other.current_index;
}

const grid_map::Index& operator *() const
{
  return polygon_indexes_[current_index];
}

PolygonIterator& operator ++()
{
  ++current_index;
  return *this;
}

[[nodiscard]] bool isPastEnd() const
{
  return current_index >= polygon_indexes_.size();
}

};
}  // namespace apparent_safe_velocity_limiter

#endif  // APPARENT_SAFE_VELOCITY_LIMITER__POLYGON_ITERATOR_HPP_

