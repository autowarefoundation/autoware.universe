// Copyright 2024 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/sat_2d.hpp"

namespace autoware::universe_utils::sat
{

namespace
{
/// @brief calculate the dot product between two points
double dot_product(const Point2d & p1, const Point2d & p2)
{
  return p1.x() * p2.x() + p1.y() * p2.y();
}

/// @brief calculate the edge normal of two points
Point2d edge_normal(const Point2d & p1, const Point2d & p2)
{
  return Point2d(p2.y() - p1.y(), p1.x() - p2.x());
}

/// @brief project a polygon onto an axis and return the minimum and maximum values
std::pair<double, double> project_polygon(const Polygon2d & poly, const Point2d & axis)
{
  double min = dot_product(poly.outer()[0], axis);
  double max = min;
  for (const auto & point : poly.outer()) {
    double projection = dot_product(point, axis);
    if (projection < min) {
      min = projection;
    }
    if (projection > max) {
      max = projection;
    }
  }
  return {min, max};
}

/// @brief check if two projections overlap
bool projections_overlap(const std::pair<double, double> & proj1, const std::pair<double, double> & proj2)
{
  return proj1.second >= proj2.first && proj2.second >= proj1.first;
}
}  // namespace

/// @brief check if two convex polygons intersect using the SAT algorithm
/// @details this function uses the Separating Axis Theorem (SAT) to determine if two convex polygons intersect. 
/// projects both polygons onto the axes defined by the normals of their edges. 
/// if the projections on any axis do not overlap, the polygons do not intersect.
/// if projections overlap on all tested axes, the function returns `true`; otherwise, it returns `false`. 
/// note that touching polygons (e.g., at a point or along an edge) will be considered as not intersecting.

bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  // check all edges of polygon1
  for (size_t i = 0; i < convex_polygon1.outer().size(); ++i) {
    size_t next_i = (i + 1) % convex_polygon1.outer().size();
    Point2d edge = edge_normal(convex_polygon1.outer()[i], convex_polygon1.outer()[next_i]);
    auto projection1 = project_polygon(convex_polygon1, edge);
    auto projection2 = project_polygon(convex_polygon2, edge);
    if (!projections_overlap(projection1, projection2)) {
      return false;
    }
  }

  // check all edges of polygon2
  for (size_t i = 0; i < convex_polygon2.outer().size(); ++i) {
    size_t next_i = (i + 1) % convex_polygon2.outer().size();
    Point2d edge = edge_normal(convex_polygon2.outer()[i], convex_polygon2.outer()[next_i]);
    auto projection1 = project_polygon(convex_polygon1, edge);
    auto projection2 = project_polygon(convex_polygon2, edge);
    if (!projections_overlap(projection1, projection2)) {
      return false;
    }
  }

  return true;  // no separating axis found, polygons must be intersecting
}

}  // namespace autoware::universe_utils::sat
