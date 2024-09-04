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

#include "autoware/universe_utils/geometry/polygon_clip.hpp"

namespace autoware_universe::utils::pc
{

/**
 * @brief Computes the dot product of two Points.
 */
double dot_product(const Point & a, const Point & b)
{
  return a.x() * b.x() + a.y() * b.y();
}

constexpr double kdoubleNearZero = 1e-7;

/**
 * @brief Checks if two line segments intersect and computes intersection parameters.
 */
bool segment_intersect(Vertex * p1, Vertex * p2, Vertex * q1, Vertex * q2, double & t1, double & t2)
{
  Point p1_q1 = p1->point - q1->point;
  Point p2_q1 = p2->point - q1->point;
  Point q2_q1 = q2->point - q1->point;

  Point q2_q1_normal(-q2_q1.y(), q2_q1.x());

  double WEC_P1 = dot_product(p1_q1, q2_q1_normal);
  double WEC_P2 = dot_product(p2_q1, q2_q1_normal);

  if (WEC_P1 * WEC_P2 >= 0.0 || WEC_P1 == 0.0 || WEC_P2 == 0.0) {
    return false;
  }

  Point p2_p1 = p2->point - p1->point;
  Point q1_p1 = q1->point - p1->point;
  Point q2_p1 = q2->point - p1->point;

  Point p2_p1_normal(-p2_p1.y(), p2_p1.x());

  double WEC_Q1 = dot_product(q1_p1, p2_p1_normal);
  double WEC_Q2 = dot_product(q2_p1, p2_p1_normal);

  if (WEC_Q1 * WEC_Q2 >= 0.0 || WEC_Q1 == 0.0 || WEC_Q2 == 0.0) {
    return false;
  }

  t1 = WEC_P1 / (WEC_P1 - WEC_P2);
  t2 = WEC_Q1 / (WEC_Q1 - WEC_Q2);

  return true;
}

/**
 * @brief Iterates over vertices of a polygon.
 */
PolygonIter::PolygonIter(const std::vector<Vertex *> & polygons)
: m_polygon(polygons), m_index(0), m_loop_end(false)
{
  if (!m_polygon.empty()) {
    m_curr_head = m_polygon.front();
    m_current = m_curr_head;
  } else {
    m_curr_head = nullptr;
    m_current = nullptr;
  }
}

/**
 * @brief Checks if there are more vertices to iterate.
 */
bool PolygonIter::has_next()
{
  return !(m_loop_end && m_index == m_polygon.size() - 1);
}

/**
 * @brief Moves to the next vertex in the iteration.
 */
void PolygonIter::move_next()
{
  if (!m_loop_end) {
    m_current = m_current->next;
    if (m_current == m_curr_head) {
      m_loop_end = true;
    }
  } else if (m_index < m_polygon.size() - 1) {
    m_index++;
    m_curr_head = m_polygon[m_index];
    m_current = m_curr_head;
    m_loop_end = false;
  }
}

/**
 * @brief Gets the current vertex.
 */
Vertex * PolygonIter::current()
{
  return m_current;
}

/**
 * @brief processes polygon operations (difference, clip, union) based on the specified operation
 * type.
 */
PolygonClip ClipAlgorithm::process_operation(
  PolygonClip subject, PolygonClip clipping, OperationType op_type)
{
  PolygonClip result;
  ClipAlgorithm algorithm(std::move(subject), std::move(clipping));

  algorithm.process_intersection();
  bool no_intersection;
  uint32_t inner_indicator;
  std::tie(no_intersection, inner_indicator) = algorithm.mark_vertices();

  auto process_vertices = [&](const std::vector<Vertex *> & vertices, bool reverse, bool is_diff) {
    for (auto vertex : vertices) {
      if (vertex->marked) continue;

      vertex->marked = true;
      std::vector<Point> pts;
      auto curr = vertex;
      bool self = true;

      pts.push_back(curr->point);

      do {
        if (!is_diff) {
          if (curr->entry_exit) {
            do {
              curr = reverse ? curr->prev : curr->next;
              pts.push_back(curr->point);
            } while (!curr->intersect);
          } else {
            do {
              curr = reverse ? curr->next : curr->prev;
              pts.push_back(curr->point);
            } while (!curr->intersect);
          }
        } else {
          if (curr->entry_exit) {
            do {
              curr = self ? curr->prev : curr->next;
              pts.push_back(curr->point);
            } while (!curr->intersect);
          } else {
            do {
              curr = self ? curr->next : curr->prev;
              pts.push_back(curr->point);
            } while (!curr->intersect);
          }
          self = !self;
        }
        curr->marked = true;
        curr = curr->neighbour;
        curr->marked = true;
      } while (curr != vertex);

      result.append_vertices(pts);
    }
  };

  if (op_type == OperationType::DIFF) {
    if (!no_intersection) {
      std::vector<Vertex *> intersection_points;
      for (auto & vert : algorithm.m_subject.m_vertex) {
        if (vert->intersect) {
          intersection_points.emplace_back(vert.get());
        }
      }

      process_vertices(intersection_points, true, true);
      return result;
    }

    if (inner_indicator == 0) {
      return PolygonClip(algorithm.m_subject);
    } else if (inner_indicator == 2) {
      return result;
    }

    return PolygonClip(algorithm.m_subject, algorithm.m_clipping, true);
  }

  if (op_type == OperationType::CLIP) {
    if (!no_intersection) {
      std::vector<Vertex *> intersection_points;
      for (auto & vert : algorithm.m_subject.m_vertex) {
        if (vert->intersect) {
          intersection_points.emplace_back(vert.get());
        }
      }

      process_vertices(intersection_points, false, false);
      return result;
    }

    if (inner_indicator == 0) {
      return result;
    } else if (inner_indicator == 1) {
      return algorithm.m_clipping;
    } else if (inner_indicator == 2) {
      return algorithm.m_subject;
    }

    return result;
  }

  if (op_type == OperationType::UNION) {
    if (!no_intersection) {
      std::vector<Vertex *> intersection_points;
      for (auto & vert : algorithm.m_subject.m_vertex) {
        if (vert->intersect) {
          intersection_points.emplace_back(vert.get());
        }
      }

      process_vertices(intersection_points, true, false);
      return result;
    }

    if (inner_indicator == 0) {
      return PolygonClip(algorithm.m_subject, algorithm.m_clipping);
    } else if (inner_indicator == 1) {
      return PolygonClip(algorithm.m_subject);
    } else {
      return PolygonClip(algorithm.m_clipping);
    }
  }

  // handle unknown OperationType
  throw std::invalid_argument("Unknown OperationType");
}

/**
 * @brief processes intersections between subject and clipping polygons
 */
void ClipAlgorithm::process_intersection()
{
  uint32_t intersection_count = 0;
  PolygonIter subj_iter(m_subject.get_vertices());

  while (subj_iter.has_next()) {
    auto current = subj_iter.current();
    PolygonIter clip_iter(m_clipping.get_vertices());
    std::vector<VertexDist> intersect_list;

    while (clip_iter.has_next()) {
      double t1 = 0.0;
      double t2 = 0.0;
      auto clip_current = clip_iter.current();

      if (segment_intersect(current, current->next, clip_current, clip_current->next, t1, t2)) {
        intersection_count++;
        auto i1 = m_subject.allocate_vertex(current, current->next, t1);
        auto i2 = m_clipping.allocate_vertex(clip_current, clip_current->next, t2);

        i1->intersect = true;
        i2->intersect = true;

        i1->neighbour = i2;
        i2->neighbour = i1;

        i2->next = clip_current->next;
        clip_current->next->prev = i2;

        clip_current->next = i2;
        i2->prev = clip_current;

        intersect_list.emplace_back(VertexDist(i1, t1));
        clip_iter.move_next();
      }

      clip_iter.move_next();
    }

    if (!intersect_list.empty()) {
      std::sort(intersect_list.begin(), intersect_list.end(), VertDistCompiler{});

      for (size_t i = 1; i < intersect_list.size(); i++) {
        auto prev = intersect_list[i - 1];
        auto next = intersect_list[i];

        prev.vert->next = next.vert;
        next.vert->prev = prev.vert;
      }
      auto head = intersect_list.front().vert;
      auto tail = intersect_list.back().vert;

      tail->next = current->next;
      current->next->prev = tail;

      current->next = head;
      head->prev = current;

      for (size_t i = 0; i < intersect_list.size(); i++) {
        subj_iter.move_next();
      }
    }
    subj_iter.move_next();
  }
  assert((intersection_count % 2) == 0);
}

/**
 * @brief marks vertices as entry or exit points and determines intersection status
 */
std::tuple<bool, uint32_t> ClipAlgorithm::mark_vertices()
{
  bool no_intersection = true;
  uint32_t inner_indicator = 0;
  bool status = false;

  PolygonIter clip_iter(m_clipping.get_vertices());

  // check if the clipping polygon's starting point is inside the subject polygon
  if (m_subject.contains(clip_iter.current()->point)) {
    status = false;
    inner_indicator = 1;
  } else {
    status = true;
    inner_indicator = 0;
  }

  // mark entry and exit points for clipping polygon
  while (clip_iter.has_next()) {
    auto current = clip_iter.current();
    if (current->intersect) {
      current->entry_exit = status;
      status = !status;
      no_intersection = false;
    }
    clip_iter.move_next();
  }

  // check if the subject polygon's starting point is inside the clipping polygon
  PolygonIter subj_iter(m_subject.get_vertices());
  if (m_clipping.contains(subj_iter.current()->point)) {
    status = false;
    inner_indicator = 2;
  } else {
    status = true;
  }

  // mark entry and exit points for subject polygon
  while (subj_iter.has_next()) {
    auto current = subj_iter.current();
    if (current->intersect) {
      current->entry_exit = status;
      status = !status;
      no_intersection = false;
    }
    subj_iter.move_next();
  }

  return std::make_tuple(no_intersection, inner_indicator);
}

/**
 * @brief constructs a polygonclip from another polygonclip instance
 */
PolygonClip::PolygonClip(const PolygonClip & other)
{
  for (auto v : other.m_sub_polygons) {
    std::vector<Point> points;
    auto p = v;
    do {
      points.push_back(p->point);
      p = p->next;
    } while (p != v);

    if (points.size() >= 3) {
      this->append_vertices(points);
    }
  }
}

/**
 * @brief constructs a polygonclip from two polygonclip instances
 */
PolygonClip::PolygonClip(const PolygonClip & p1, const PolygonClip & p2, bool p2_reserve)
{
  // Process sub-polygons from p1
  for (auto v : p1.m_sub_polygons) {
    std::vector<Point> points;
    auto p = v;
    do {
      points.push_back(p->point);
      p = p->next;
    } while (p != v);

    if (points.size() >= 3) {
      this->append_vertices(points);
    }
  }

  // Process sub-polygons from p2
  for (auto v : p2.m_sub_polygons) {
    std::vector<Point> points;
    auto p = v;
    do {
      points.push_back(p->point);
      p = p2_reserve ? p->prev : p->next;
    } while (p != v);

    if (points.size() >= 3) {
      this->append_vertices(points);
    }
  }
}

/**
 * @brief appends a closed loop of vertices to the polygonclip
 */
void PolygonClip::append_vertices(const std::vector<Point> & points)
{
  if (points.size() < 3) {
    return;
  }
  auto head = allocate_vertex(points.front());
  auto prev = head;
  for (size_t i = 1; i < points.size(); i++) {
    auto next = allocate_vertex(points[i]);
    prev->next = next;
    next->prev = prev;
    prev = next;
  }
  prev->next = head;
  head->prev = prev;
  m_sub_polygons.emplace_back(head);
}

/**
 * @brief checks if a point is inside the polygon
 */
bool PolygonClip::contains(const Point & p) const
{
  bool contains = false;
  int32_t winding_num = 0;
  PolygonIter iter(m_sub_polygons);
  constexpr double tolerance = 1e-9;

  while (iter.has_next()) {
    auto curr = iter.current();
    auto next = curr->next;
    bool y_intersects = ((next->point.y() > p.y()) != (curr->point.y() > p.y())) &&
                        (p.x() < (curr->point.x() - next->point.x()) * (p.y() - next->point.y()) /
                                     (curr->point.y() - next->point.y()) +
                                   next->point.x());

    if (y_intersects) {
      contains = !contains;

      if (
        std::abs(curr->point.x() - next->point.x()) < tolerance &&
        std::abs(curr->point.y() - next->point.y()) < tolerance) {
        iter.move_next();
        continue;
      }

      if (curr->point.x() < next->point.x() - tolerance) {
        winding_num += 1;
      } else if (curr->point.x() > next->point.x() + tolerance) {
        winding_num -= 1;
      }
    }

    iter.move_next();
  }

  return contains;
}

/**
 * @brief allocates a new vertex for a given point
 */
Vertex * PolygonClip::allocate_vertex(const Point & p)
{
  if (!m_left_top) {
    m_left_top.emplace(p.pt);
  } else {
    if (m_left_top->x() >= p.x()) {
      m_left_top->pt = {p.pt.x(), m_left_top->pt.y()};
    }
    if (m_left_top->y() >= p.y()) {
      m_left_top->pt = {m_left_top->pt.x(), p.pt.y()};
    }
  }

  if (!m_right_bottom) {
    m_right_bottom.emplace(p.pt);
  } else {
    if (m_right_bottom->x() < p.x()) {
      m_right_bottom->pt = {p.pt.x(), m_right_bottom->pt.y()};
    }
    if (m_right_bottom->y() < p.y()) {
      m_right_bottom->pt = {m_right_bottom->pt.x(), p.pt.y()};
    }
  }

  m_vertex.emplace_back(std::make_unique<Vertex>(p));
  return m_vertex.back().get();
}

/**
 * @brief allocates a new vertex between two vertices
 */
Vertex * PolygonClip::allocate_vertex(Vertex * p1, Vertex * p2, double t)
{
  Point p = (p1->point * (1.0 - t)) + (p2->point * t);
  return allocate_vertex(p);
}

/**
 * @brief clips a polygon against another polygon
 */
PolygonClip PolygonClip::Clip(const PolygonClip & subject, const PolygonClip & clipping)
{
  return ClipAlgorithm::process_operation(subject, clipping, OperationType::CLIP);
}

/**
 * @brief performs a union operation between two polygons
 */
PolygonClip PolygonClip::Union(const PolygonClip & subject, const PolygonClip & clipping)
{
  return ClipAlgorithm::process_operation(subject, clipping, OperationType::UNION);
}

/**
 * @brief performs a difference operation between two polygons
 */
PolygonClip PolygonClip::Diff(const PolygonClip & subject, const PolygonClip & clipping)
{
  return ClipAlgorithm::process_operation(subject, clipping, OperationType::DIFF);
}

}  // namespace autoware_universe::utils::pc
