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

#include "autoware/universe_utils/geometry/ear_clipping.hpp"

#ifndef EAR_CUT_IMPL_HPP
#define EAR_CUT_IMPL_HPP

namespace autoware::universe_utils
{

std::size_t construct_point(std::size_t index, const Point2d & point, std::vector<Point> & points)
{
  points.emplace_back(index, point);
  return index;
}

std::vector<Point> perform_triangulation(
  const Polygon2d & polygon, std::vector<std::size_t> & indices)
{
  indices.clear();
  std::vector<Point> points_;
  std::size_t vertices = 0;
  const auto & outer_ring = polygon.outer();
  std::size_t len = outer_ring.size();
  points_.reserve(len * 3 / 2);

  if (polygon.outer().empty()) return points_;

  indices.reserve(len + outer_ring.size());
  auto outer_point = linked_list(outer_ring, true, vertices, points_);
  if (outer_point == points_[outer_point.value()].prev_index.value()) {
    return points_;
  }

  if (!polygon.inners().empty()) {
    outer_point = eliminate_holes(polygon.inners(), outer_point.value(), vertices, points_);
  }

  ear_clipping_linked(outer_point.value(), indices, points_);
  return points_;
}

/// @brief create a Point and optionally link it with the previous one (in a circular doubly linked
/// list)
std::size_t insert_point(
  std::size_t i, const Point2d & pt, std::vector<Point> & points_vec,
  std::optional<std::size_t> last_index)
{
  std::size_t p_idx = points_vec.size();
  points_vec.emplace_back(i, pt);
  if (!last_index.has_value()) {
    points_vec[p_idx].prev_index = p_idx;
    points_vec[p_idx].next_index = p_idx;
  } else {
    std::size_t last = last_index.value();
    std::size_t next = points_vec[last].next_index.value();
    points_vec[p_idx].prev_index = last;
    points_vec[p_idx].next_index = next;

    points_vec[last].next_index = p_idx;
    points_vec[next].prev_index = p_idx;
  }

  return p_idx;
}

std::optional<std::size_t> linked_list(
  const LinearRing2d & points, bool clockwise, std::size_t & vertices,
  std::vector<Point> & points_vec)
{
  double sum = 0;
  const std::size_t len = points.size();
  std::optional<std::size_t> last_index = std::nullopt;

  for (std::size_t i = 0, j = len > 0 ? len - 1 : 0; i < len; j = i++) {
    const auto & p1 = points[i];
    const auto & p2 = points[j];
    sum += (p2.x() - p1.x()) * (p1.y() + p2.y());
  }

  if (clockwise == (sum > 0)) {
    for (std::size_t i = 0; i < len; i++) {
      last_index = insert_point(vertices + i, points[i], points_vec, last_index);
    }
  } else {
    for (size_t i = len; i-- > 0;) {
      last_index = insert_point(vertices + i, points[i], points_vec, last_index);
    }
  }

  if (
    last_index.has_value() &&
    equals(last_index.value(), points_vec[last_index.value()].next_index.value(), points_vec)) {
    std::size_t next_index = points_vec[last_index.value()].next_index.value();
    remove_point(last_index.value(), points_vec);
    last_index = next_index;
  }
  vertices += len;
  return last_index;
}

/// @brief eliminate colinear or duplicate points
std::size_t filter_points(
  std::size_t start_index, std::optional<std::size_t> end_index, std::vector<Point> & points_vec)
{
  if (!end_index.has_value()) {
    end_index = start_index;
  }

  auto p = start_index;
  bool again = false;
  do {
    again = false;
    if (
      !points_vec[p].steiner &&
      (equals(p, points_vec[p].next_index.value(), points_vec) ||
       area(points_vec, points_vec[p].prev_index.value(), p, points_vec[p].next_index.value()) ==
         0)) {
      remove_point(p, points_vec);
      p = points_vec[p].prev_index.value();

      if (p == points_vec[p].next_index.value()) break;
      again = true;

    } else {
      p = points_vec[p].next_index.value();
    }
  } while (again || p != end_index.value());

  return end_index.value();
}

/// @brief find a bridge between vertices that connects hole with an outer ring and link it
std::size_t eliminate_hole(
  std::size_t hole_index, std::size_t outer_index, std::vector<Point> & points_vec)
{
  auto bridge = find_hole_bridge(hole_index, outer_index, points_vec);
  if (!bridge.has_value()) {
    return outer_index;
  }
  auto bridge_reverse = split_polygon(bridge.value(), hole_index, points_vec);

  filter_points(bridge_reverse, points_vec[bridge_reverse].next_index.value(), points_vec);
  return filter_points(bridge.value(), points_vec[bridge.value()].next_index.value(), points_vec);
}

std::size_t eliminate_holes(
  const std::vector<LinearRing2d> & inners, std::size_t outer_index, std::size_t & vertices,
  std::vector<Point> & points_vec)
{
  const std::size_t len = inners.size();

  std::vector<std::size_t> queue;
  for (std::size_t i = 0; i < len; i++) {
    auto list = linked_list(inners[i], false, vertices, points_vec);

    if (list.has_value()) {
      if (points_vec[list.value()].next_index == list.value()) {
        points_vec[list.value()].steiner = true;
      }

      queue.push_back(get_leftmost(list.value(), points_vec));
    }
  }

  std::sort(queue.begin(), queue.end(), [&](std::size_t a, std::size_t b) {
    return points_vec[a].x() < points_vec[b].x();
  });

  for (const auto & q : queue) {
    outer_index = eliminate_hole(q, outer_index, points_vec);
  }

  return outer_index;
}

/// @brief triangulate a polygon using linked list
void ear_clipping_linked(
  std::optional<std::size_t> ear_index, std::vector<std::size_t> & indices,
  std::vector<Point> & points_vec, int pass)
{
  if (!ear_index.has_value()) {
    return;
  }

  auto stop = ear_index;  // Stopping point for the loop
  std::optional<std::size_t> next = std::nullopt;

  // Iterate through ears, slicing them one by one
  while (points_vec[ear_index.value()].prev_index.value() !=
         points_vec[ear_index.value()].next_index.value()) {
    next = points_vec[ear_index.value()].next_index;

    if (is_ear(ear_index.value(), points_vec)) {
      // Cut off the triangle and store the indices
      indices.emplace_back(points_vec[ear_index.value()].prev_index.value());
      indices.emplace_back(ear_index.value());
      indices.emplace_back(next.value());

      // Remove the ear and update the ear_index to continue
      remove_point(ear_index.value(), points_vec);

      // Move to the next ear after removal
      ear_index = points_vec[next.value()].next_index.value();
      stop = points_vec[next.value()].next_index.value();
      continue;
    }

    ear_index = next.value();

    if (ear_index == stop) {
      if (pass == 0) {
        ear_clipping_linked(
          filter_points(ear_index.value(), std::nullopt, points_vec), indices, points_vec, 1);
      } else if (pass == 1) {
        ear_index = cure_local_intersections(
          filter_points(ear_index.value(), std::nullopt, points_vec), indices, points_vec);
        ear_clipping_linked(ear_index.value(), indices, points_vec, 2);
      } else if (pass == 2) {
        split_ear_clipping(points_vec, ear_index.value(), indices);
      }
      break;
    }
  }
}

/// @brief check whether ear is valid
bool is_ear(std::size_t ear_index, const std::vector<Point> & points_vec)
{
  const auto a_index = points_vec[ear_index].prev_index.value();
  const auto b_index = ear_index;
  const auto c_index = points_vec[ear_index].next_index.value();

  const auto a = points_vec[a_index];
  const auto b = points_vec[b_index];
  const auto c = points_vec[c_index];

  if (area(points_vec, a_index, b_index, c_index) >= 0) return false;  // Reflex, can't be an ear

  auto p_index = points_vec[c_index].next_index.value();
  while (p_index != a_index) {
    const auto p = points_vec[p_index];
    if (
      point_in_triangle(a.x(), a.y(), b.x(), b.y(), c.x(), c.y(), p.x(), p.y()) &&
      area(points_vec, p.prev_index.value(), p_index, p.next_index.value()) >= 0) {
      return false;
    }
    p_index = points_vec[p_index].next_index.value();
  }

  return true;
}

/// @brief go through all polygon Points and cure small local self-intersections
std::size_t cure_local_intersections(
  std::size_t start_index, std::vector<std::size_t> & indices, std::vector<Point> & points_vec)
{
  auto p = start_index;
  bool updated = false;

  do {
    auto a = points_vec[p].prev_index.value();
    auto b = points_vec[points_vec[p].next_index.value()].next_index.value();

    if (
      !equals(a, b, points_vec) &&
      intersects(a, p, points_vec[p].next_index.value(), b, points_vec) &&
      locally_inside(a, b, points_vec) && locally_inside(b, a, points_vec)) {
      indices.emplace_back(a);
      indices.emplace_back(p);
      indices.emplace_back(b);

      remove_point(p, points_vec);
      remove_point(points_vec[p].next_index.value(), points_vec);

      p = start_index = b;
      updated = true;
    } else {
      p = points_vec[p].next_index.value();
    }
  } while (p != start_index && updated);

  return filter_points(p, std::nullopt, points_vec);
}

// cspell: ignore Eberly
/// @brief David Eberly's algorithm for finding a bridge between hole and outer polygon using vector
/// indices
std::optional<std::size_t> find_hole_bridge(
  std::size_t hole_index, std::size_t outer_point_index, const std::vector<Point> & points)
{
  std::size_t p = outer_point_index;
  double hx = points[hole_index].x();
  double hy = points[hole_index].y();
  double qx = -std::numeric_limits<double>::infinity();
  std::optional<std::size_t> bridge_point = std::nullopt;
  do {
    std::size_t next_index = points[p].next_index.value();
    if (
      hy <= points[p].y() && hy >= points[next_index].y() &&
      points[next_index].y() != points[p].y()) {
      double x = points[p].x() + (hy - points[p].y()) * (points[next_index].x() - points[p].x()) /
                                   (points[next_index].y() - points[p].y());
      if (x <= hx && x > qx) {
        qx = x;
        bridge_point = points[p].x() < points[next_index].x() ? p : next_index;
        if (x == hx) return bridge_point.value();
      }
    }
    p = next_index;
  } while (p != outer_point_index);

  if (!bridge_point.has_value()) return bridge_point.value();

  const std::size_t stop = bridge_point.value();
  double min_tan = std::numeric_limits<double>::infinity();

  p = bridge_point.value();
  double mx = points[bridge_point.value()].x();
  double my = points[bridge_point.value()].y();

  do {
    if (
      hx >= points[p].x() && points[p].x() >= mx && hx != points[p].x() &&
      point_in_triangle(
        hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, points[p].x(), points[p].y())) {
      double current_tan = std::abs(hy - points[p].y()) / (hx - points[p].x());

      if (
        locally_inside(p, hole_index, points) &&
        (current_tan < min_tan ||
         (current_tan == min_tan && (points[p].x() > points[bridge_point.value()].x() ||
                                     sector_contains_sector(bridge_point.value(), p, points))))) {
        bridge_point = p;
        min_tan = current_tan;
      }
    }

    p = points[p].next_index.value();
  } while (p != stop);

  return bridge_point;
}

/// @brief split the polygon using ear clipping
void split_ear_clipping(
  std::vector<Point> & points, std::size_t start_idx, std::vector<std::size_t> & indices)
{
  std::size_t a_idx = start_idx;
  do {
    std::size_t b_idx = points[points[a_idx].next_index.value()].next_index.value();
    while (b_idx != points[a_idx].prev_index.value()) {
      if (a_idx != b_idx && is_valid_diagonal(a_idx, b_idx, points)) {
        std::size_t c_idx = split_polygon(a_idx, b_idx, points);

        a_idx = filter_points(start_idx, points[a_idx].next_index.value(), points);
        c_idx = filter_points(start_idx, points[c_idx].next_index.value(), points);

        ear_clipping_linked(a_idx, indices, points);
        ear_clipping_linked(c_idx, indices, points);
        return;
      }
      b_idx = points[b_idx].next_index.value();
    }
    a_idx = points[a_idx].next_index.value();
  } while (a_idx != start_idx);
}

/// @brief check whether the sector in vertex m contains the sector in vertex p in the same
/// coordinates
bool sector_contains_sector(std::size_t m_idx, std::size_t p_idx, const std::vector<Point> & points)
{
  return area(points, points[m_idx].prev_index.value(), m_idx, p_idx) < 0 &&
         area(points, p_idx, points[m_idx].next_index.value(), m_idx) < 0;
}

/// @brief find the leftmost Point of a polygon ring
std::size_t get_leftmost(std::size_t start_idx, const std::vector<Point> & points)
{
  std::size_t p_idx = start_idx;
  std::size_t left_most_idx = start_idx;
  do {
    if (
      points[p_idx].x() < points[left_most_idx].x() ||
      (points[p_idx].x() == points[left_most_idx].x() &&
       points[p_idx].y() < points[left_most_idx].y())) {
      left_most_idx = p_idx;
    }
    p_idx = points[p_idx].next_index.value();
  } while (p_idx != start_idx);

  return left_most_idx;
}

/// @brief check if a point lies within a convex triangle
bool point_in_triangle(
  double ax, double ay, double bx, double by, double cx, double cy, double px, double py)
{
  return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
         (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
         (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

/// @brief check if a diagonal between two polygon Points is valid
bool is_valid_diagonal(std::size_t a_idx, std::size_t b_idx, const std::vector<Point> & points)
{
  // Extract indices for the next and previous points
  std::size_t a_next_idx = points[a_idx].next_index.value();
  std::size_t a_prev_idx = points[a_idx].prev_index.value();
  std::size_t b_next_idx = points[b_idx].next_index.value();
  std::size_t b_prev_idx = points[b_idx].prev_index.value();

  // Check if diagonal does not connect adjacent or same points
  if (a_next_idx == b_idx || a_prev_idx == b_idx || intersects_polygon(points, a_idx, b_idx)) {
    return false;
  }

  // Check if diagonals are locally inside and not intersecting
  bool is_locally_inside_ab = locally_inside(a_idx, b_idx, points);
  bool is_locally_inside_ba = locally_inside(b_idx, a_idx, points);
  bool is_middle_inside = middle_inside(a_idx, b_idx, points);

  bool is_valid_diagonal =
    (is_locally_inside_ab && is_locally_inside_ba && is_middle_inside &&
     (area(points, a_prev_idx, a_idx, b_prev_idx) != 0.0 ||
      area(points, a_idx, b_prev_idx, b_idx) != 0.0)) ||
    (equals(a_idx, b_idx, points) && area(points, a_prev_idx, a_idx, a_next_idx) > 0 &&
     area(points, b_prev_idx, b_idx, b_next_idx) > 0);

  return is_valid_diagonal;
}

/// @brief signed area of a triangle
double area(
  const std::vector<Point> & points, std::size_t p_idx, std::size_t q_idx, std::size_t r_idx)
{
  const Point & p = points[p_idx];
  const Point & q = points[q_idx];
  const Point & r = points[r_idx];
  return (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
}

/// @brief check if two points are equal
bool equals(std::size_t p1_idx, std::size_t p2_idx, const std::vector<Point> & points)
{
  return points[p1_idx].x() == points[p2_idx].x() && points[p1_idx].y() == points[p2_idx].y();
}

/// @brief check if two segments intersect
bool intersects(
  std::size_t p1_idx, std::size_t q1_idx, std::size_t p2_idx, std::size_t q2_idx,
  const std::vector<Point> & points)
{
  int o1 = sign(area(points, p1_idx, q1_idx, p2_idx));
  int o2 = sign(area(points, p1_idx, q1_idx, q2_idx));
  int o3 = sign(area(points, p2_idx, q2_idx, p1_idx));
  int o4 = sign(area(points, p2_idx, q2_idx, q1_idx));

  if (o1 != o2 && o3 != o4) return true;

  if (o1 == 0 && on_segment(points, p1_idx, p2_idx, q1_idx)) return true;
  if (o2 == 0 && on_segment(points, p1_idx, q2_idx, q1_idx)) return true;
  if (o3 == 0 && on_segment(points, p2_idx, p1_idx, q2_idx)) return true;
  if (o4 == 0 && on_segment(points, p2_idx, q1_idx, q2_idx)) return true;

  return false;
}

/// @brief for collinear points p, q, r, check if point q lies on segment pr
bool on_segment(
  const std::vector<Point> & points, std::size_t p_idx, std::size_t q_idx, std::size_t r_idx)
{
  const Point & p = points[p_idx];
  const Point & q = points[q_idx];
  const Point & r = points[r_idx];
  return q.x() <= std::max<double>(p.x(), r.x()) && q.x() >= std::min<double>(p.x(), r.x()) &&
         q.y() <= std::max<double>(p.y(), r.y()) && q.y() >= std::min<double>(p.y(), r.y());
}

/// @brief sign function for area calculation
int sign(double val)
{
  return (0.0 < val) - (val < 0.0);
}

/// @brief check if a polygon diagonal intersects any polygon segments
bool intersects_polygon(const std::vector<Point> & points, std::size_t a_idx, std::size_t b_idx)
{
  std::size_t p_idx = a_idx;
  do {
    std::size_t p_next_idx = points[p_idx].next_index.value();  // Get the next point in the polygon

    if (p_idx != a_idx && p_next_idx != a_idx && p_idx != b_idx && p_next_idx != b_idx) {
      if (intersects(p_idx, p_next_idx, a_idx, b_idx, points)) {
        return true;
      }
    }

    p_idx = p_next_idx;
  } while (p_idx != a_idx);

  return false;
}

/// @brief check if a polygon diagonal is locally inside the polygon
bool locally_inside(std::size_t a_idx, std::size_t b_idx, const std::vector<Point> & points)
{
  return area(points, points[a_idx].prev_index.value(), a_idx, points[a_idx].next_index.value()) < 0
           ? area(points, a_idx, b_idx, points[a_idx].next_index.value()) >= 0 &&
               area(points, a_idx, points[a_idx].prev_index.value(), b_idx) >= 0
           : area(points, a_idx, b_idx, points[a_idx].prev_index.value()) < 0 ||
               area(points, a_idx, points[a_idx].next_index.value(), b_idx) < 0;
}

/// @brief check if the middle vertex of a polygon diagonal is inside the polygon
bool middle_inside(std::size_t a_idx, std::size_t b_idx, const std::vector<Point> & points)
{
  std::size_t p_idx = a_idx;
  bool inside = false;
  double px = (points[a_idx].x() + points[b_idx].x()) / 2;
  double py = (points[a_idx].y() + points[b_idx].y()) / 2;
  do {
    if (
      ((points[p_idx].y() > py) != (points[points[p_idx].next_index.value()].y() > py)) &&
      points[points[p_idx].next_index.value()].y() != points[p_idx].y() &&
      (px < (points[points[p_idx].next_index.value()].x() - points[p_idx].x()) *
                (py - points[p_idx].y()) /
                (points[points[p_idx].next_index.value()].y() - points[p_idx].y()) +
              points[p_idx].x())) {
      inside = !inside;
    }
    p_idx = points[p_idx].next_index.value();
  } while (p_idx != a_idx);

  return inside;
}

/// @brief link two polygon vertices with a bridge
std::size_t split_polygon(std::size_t a_index, std::size_t b_index, std::vector<Point> & points)
{
  Point2d a_point(points[a_index].x(), points[a_index].y());
  Point2d b_point(points[b_index].x(), points[b_index].y());

  std::size_t a2_idx = construct_point(a_index, a_point, points);
  std::size_t b2_idx = construct_point(b_index, b_point, points);

  std::size_t an_idx = points[a_index].next_index.value();
  std::size_t bp_idx = points[b_index].prev_index.value();

  points[a_index].next_index = b_index;
  points[b_index].prev_index = a_index;

  points[a2_idx].next_index = an_idx;
  if (an_idx != a_index) {
    points[an_idx].prev_index = a2_idx;
  }

  points[b2_idx].next_index = a2_idx;
  points[a2_idx].prev_index = b2_idx;

  if (bp_idx != b_index) {
    points[bp_idx].next_index = b2_idx;
  }
  points[b2_idx].prev_index = bp_idx;

  return b2_idx;
}

/// @brief remove a Point from the linked list
void remove_point(std::size_t p_index, std::vector<Point> & points)
{
  std::size_t prev_index = points[p_index].prev_index.value();
  std::size_t next_index = points[p_index].next_index.value();

  points[prev_index].next_index = next_index;
  points[next_index].prev_index = prev_index;
}

/// @brief main triangulate function
std::vector<Polygon2d> triangulate(const Polygon2d & poly)
{
  std::vector<std::size_t> indices;
  auto list_ = perform_triangulation(poly, indices);

  std::vector<Polygon2d> triangles;
  const std::size_t num_indices = indices.size();

  if (num_indices % 3 != 0) {
    throw std::runtime_error("Indices size should be a multiple of 3");
  }

  for (std::size_t i = 0; i < num_indices; i += 3) {
    Polygon2d triangle;
    triangle.outer().push_back(list_[indices[i]].pt);
    triangle.outer().push_back(list_[indices[i + 1]].pt);
    triangle.outer().push_back(list_[indices[i + 2]].pt);
    triangle.outer().push_back(list_[indices[i]].pt);

    triangles.push_back(triangle);
  }
  list_.clear();
  return triangles;
}

}  // namespace autoware::universe_utils

#endif  // EAR_CUT_IMPL_HPP
