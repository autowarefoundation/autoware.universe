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

#include "autoware/universe_utils/geometry/gjk_2d.hpp"

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <boost/geometry/algorithms/equals.hpp>

namespace autoware::universe_utils::gjk
{

namespace
{

double dot_product(const Point2d & p1, const Point2d & p2)
{
  return p1.x() * p2.x() + p1.y() * p2.y();
}

size_t furthest_vertex_idx(const Polygon2d & poly, const Point2d & direction)
{
  auto furthest_distance = dot_product(poly.outer()[0], direction);
  size_t furthest_idx = 0UL;
  for (auto i = 1UL; i < poly.outer().size(); ++i) {
    const auto distance = dot_product(poly.outer()[i], direction);
    if (distance > furthest_distance) {
      furthest_distance = distance;
      furthest_idx = i;
    }
  }
  return furthest_idx;
}

Point2d support_vertex(const Polygon2d & poly1, const Polygon2d & poly2, const Point2d & direction)
{
  const auto opposite_direction = Point2d(-direction.x(), -direction.y());
  const auto idx1 = furthest_vertex_idx(poly1, direction);
  const auto idx2 = furthest_vertex_idx(poly2, opposite_direction);
  return Point2d(
    poly1.outer()[idx1].x() - poly2.outer()[idx2].x(),
    poly1.outer()[idx1].y() - poly2.outer()[idx2].y());
}

bool same_direction(const Point2d & p1, const Point2d & p2)
{
  return dot_product(p1, p2) > 0.0;
}

Point2d cross_product(const Point2d & p1, const Point2d & p2, const Point2d & p3)
{
  const auto tmp = p1.x() * p2.y() - p1.y() * p2.x();
  return Point2d(-p3.y() * tmp, p3.x() * tmp);
}
}  // namespace

bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  if (convex_polygon1.outer().empty() || convex_polygon2.outer().empty()) {
    return false;
  }
  if (boost::geometry::equals(convex_polygon1, convex_polygon2)) {
    return true;
  }

  Point2d direction = {1.0, 0.0};
  Point2d a = support_vertex(convex_polygon1, convex_polygon2, direction);
  direction = {-a.x(), -a.y()};
  Point2d b = support_vertex(convex_polygon1, convex_polygon2, direction);
  if (dot_product(b, direction) <= 0.0) {
    return false;
  }
  if (a.isZero() || b.isZero()) {  // the 2 polygons intersect at a vertex
    return true;
  }
  Point2d ab = {b.x() - a.x(), b.y() - a.y()};
  Point2d ao = {-a.x(), -a.y()};
  // Prepare loop variables to not recreate them at each iteration
  Point2d c;
  Point2d co;
  Point2d ca;
  Point2d cb;
  Point2d ca_perpendicular;
  Point2d cb_perpendicular;
  direction = cross_product(ab, ao, ab);
  while (true) {
    c = support_vertex(convex_polygon1, convex_polygon2, direction);
    if (c.isZero()) {  // the 2 polygons intersect at a vertex
      return true;
    }
    if (!same_direction(c, direction)) {
      return false;
    }
    co.x() = -c.x();
    co.y() = -c.y();
    ca.x() = a.x() - c.x();
    ca.y() = a.y() - c.y();
    cb.x() = b.x() - c.x();
    cb.y() = b.y() - c.y();
    ca_perpendicular = cross_product(cb, ca, ca);
    cb_perpendicular = cross_product(ca, cb, cb);
    if (same_direction(ca_perpendicular, co)) {
      b.x() = c.x();
      b.y() = c.y();
      direction.x() = ca_perpendicular.x();
      direction.y() = ca_perpendicular.y();
    } else if (same_direction(cb_perpendicular, co)) {
      a.x() = c.x();
      a.y() = c.y();
      direction.x() = cb_perpendicular.x();
      direction.y() = cb_perpendicular.y();
    } else {
      return true;
    }
  }
  return true;
}
}  // namespace autoware::universe_utils::gjk
