// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EARCLIPPING_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EARCLIPPING_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <vector>

namespace autoware::universe_utils
{

class earclipping
{
public:
  std::vector<std::size_t> indices;
  std::size_t vertices = 0;
  using Polygon2d = autoware::universe_utils::Polygon2d;
  using Point2d = autoware::universe_utils::Point2d;
  using LinearRing2d = autoware::universe_utils::LinearRing2d;

  void operator()(const Polygon2d & points);

private:
  struct Point
  {
    Point(std::size_t index, const Point2d & point) : i(index), pt(point) {}

    Point(const Point &) = delete;
    Point & operator=(const Point &) = delete;
    Point(Point &&) = delete;
    Point & operator=(Point &&) = delete;

    const std::size_t i;  // Index of the point in the original polygon
    const Point2d pt;     // The Point2d object representing the coordinates

    // Previous and next vertices (Points) in the polygon ring
    Point * prev = nullptr;
    Point * next = nullptr;
    bool steiner = false;

    double x() const { return pt.x(); }
    double y() const { return pt.y(); }
  };

  // Use std::vector instead of ObjectPool
  std::vector<Point *> Points;

  Point * linked_list(const LinearRing2d & points, bool clockwise);
  Point * filter_points(Point * start, Point * end = nullptr);
  Point * cure_local_intersections(Point * start);
  Point * get_leftmost(Point * start);
  Point * split_polygon(Point * a, Point * b);
  Point * insert_point(std::size_t i, const Point2d & p, Point * last);
  Point * eliminate_holes(const std::vector<LinearRing2d> & points, Point * outer_point);
  Point * eliminate_hole(Point * hole, Point * outer_point);
  Point * find_hole_bridge(Point * hole, Point * outer_point);
  void earclipping_linked(Point * ear, int pass = 0);
  void split_earclipping(Point * start);
  void remove_point(Point * p);
  bool is_ear(Point * ear);
  bool sector_contains_sector(const Point * m, const Point * p);
  bool point_in_triangle(
    double ax, double ay, double bx, double by, double cx, double cy, double px, double py) const;
  bool is_valid_diagonal(Point * a, Point * b);
  bool equals(const Point * p1, const Point * p2);
  bool intersects(const Point * p1, const Point * q1, const Point * p2, const Point * q2);
  bool on_segment(const Point * p, const Point * q, const Point * r);
  bool intersects_polygon(const Point * a, const Point * b);
  bool locally_inside(const Point * a, const Point * b);
  bool middle_inside(const Point * a, const Point * b);
  int sign(double val);
  double area(const Point * p, const Point * q, const Point * r) const;

  // Function to construct a new Point object
  earclipping::Point * construct_point(std::size_t index, const Point2d & point)
  {
    Point * new_point = new Point(index, point);
    Points.push_back(new_point);
    return new_point;
  }
};

/// @brief Triangulate based on earclipping algorithm
/// @param polyogn concave/convex polygon with/without holes
/// @details algorithm based on https://github.com/mapbox/earclipping with modification
std::vector<autoware::universe_utils::Polygon2d> triangulate(
  const autoware::universe_utils::Polygon2d & poly);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EARCLIPPING_HPP_
