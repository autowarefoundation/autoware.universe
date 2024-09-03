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

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <set>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include "autoware/universe_utils/geometry/polygon_clip.hpp"

using namespace std;

namespace autoware_universe::utils::pc {

struct VertexDist {
  Vertex *vert;
  double t;

  VertexDist(Vertex *vert, double t) : vert(vert), t(t) {}
};

struct VertDistCompiler {
  bool operator()(const VertexDist &v1, const VertexDist &v2) {
    return v1.t < v2.t;
  }
};

// Dot product of two Points
double dot_product(const Point &a, const Point &b) {
    return a.x() * b.x() + a.y() * b.y();
}

constexpr double kdoubleNearZero = 1e-7;

bool Math::segment_intersect(Vertex *p1, Vertex *p2, Vertex *q1, Vertex *q2,
                             double &t1, double &t2) {
    Point2d p1_q1(p1->point.pt.x() - q1->point.pt.x(), p1->point.pt.y() - q1->point.pt.y());
    Point2d p2_q1(p2->point.pt.x() - q1->point.pt.x(), p2->point.pt.y() - q1->point.pt.y());
    Point2d q2_q1(q2->point.pt.x() - q1->point.pt.x(), q2->point.pt.y() - q1->point.pt.y());

    Point2d q2_q1_normal(-q2_q1.y(), q2_q1.x());

    double WEC_P1 = dot_product(p1_q1, q2_q1_normal);
    double WEC_P2 = dot_product(p2_q1, q2_q1_normal);

    if (WEC_P1 == 0.0 || WEC_P2 == 0.0) {
        return false;
    }

    if (WEC_P1 * WEC_P2 >= 0.0) {
        return false;
    }

    Point2d p2_p1(p2->point.pt.x() - p1->point.pt.x(), p2->point.pt.y() - p1->point.pt.y());
    Point2d q1_p1(q1->point.pt.x() - p1->point.pt.x(), q1->point.pt.y() - p1->point.pt.y());
    Point2d q2_p1(q2->point.pt.x() - p1->point.pt.x(), q2->point.pt.y() - p1->point.pt.y());

    Point2d p2_p1_normal(-p2_p1.y(), p2_p1.x());

    double WEC_Q1 = dot_product(q1_p1, p2_p1_normal);
    double WEC_Q2 = dot_product(q2_p1, p2_p1_normal);

    if (WEC_Q1 == 0.0 || WEC_Q2 == 0.0) {
        return false;
    }

    if (WEC_Q1 * WEC_Q2 >= 0.0) {
        return false;
    }

    t1 = WEC_P1 / (WEC_P1 - WEC_P2);
    t2 = WEC_Q1 / (WEC_Q1 - WEC_Q2);

    return true;
}


PolygonIter::PolygonIter(const std::vector<Vertex *> &polygons)
    : m_polygon(polygons) {
  m_index = 0;
  if (m_polygon.empty()) {
    m_curr_head = nullptr;
    m_current = nullptr;
  } else {
    m_curr_head = m_polygon.front();
    m_current = m_curr_head;
  }
}

bool PolygonIter::has_next() {
  if (m_loop_end && m_index == m_polygon.size() - 1) {
    return false;
  }

  return true;
}

void PolygonIter::move_next() {
  if (!m_loop_end) {
    m_current = m_current->next;
    if (m_current == m_curr_head) {
      m_loop_end = true;
    }
    return;
  }

  if (m_index == m_polygon.size() - 1) {
    return;
  }

  m_index++;
  m_curr_head = m_polygon[m_index];
  m_current = m_curr_head;
  m_loop_end = false;
}

Vertex *PolygonIter::current() { return m_current; }


PolygonClip ClipAlgorithm::do_clip(PolygonClip subject, PolygonClip clipping) {
  PolygonClip result;

  ClipAlgorithm algorithm(std::move(subject), std::move(clipping));

  algorithm.process_intersection();

  bool no_intersection;
  uint32_t inner_indicator;

  std::tie(no_intersection, inner_indicator) = algorithm.mark_vertices();

  if (no_intersection) {
    if (inner_indicator == 0) {
      return result;
    } else if (inner_indicator == 1) {
      return algorithm.m_clipping;
    } else if (inner_indicator == 2) {
      return algorithm.m_subject;
    }

    return result;
  }

  std::vector<Vertex *> intersection_points;
  for (auto &vert : algorithm.m_subject.m_vertex) {
    if (vert->intersect) {
      intersection_points.emplace_back(vert.get());
    }
  }

  for (auto vert : intersection_points) {
    if (vert->marked) {
      continue;
    }

    std::vector<Point> pts;

    vert->marked = true;

    auto start = vert;
    auto current = start;

    pts.emplace_back(current->point);

    do {
      if (current->entry_exit) {
        do {
          current = current->next;

          pts.emplace_back(current->point);
        } while (!current->intersect);
      } else {
        do {
          current = current->prev;

          pts.emplace_back(current->point);
        } while (!current->intersect);
      }
      current->marked = true;
      current = current->neighbour;
      current->marked = true;
    } while (current != start);

    result.append_vertices(pts);
  }

  return result;
}

PolygonClip ClipAlgorithm::do_union(PolygonClip subject, PolygonClip clipping) {
  PolygonClip result;

  ClipAlgorithm algorithm(std::move(subject), std::move(clipping));

  algorithm.process_intersection();

  bool no_intersection;
  uint32_t inner_indicator;

  std::tie(no_intersection, inner_indicator) = algorithm.mark_vertices();

  if (!no_intersection) {
    std::vector<Vertex *> intersection_points;
    for (auto &vert : algorithm.m_subject.m_vertex) {
      if (vert->intersect) {
        intersection_points.emplace_back(vert.get());
      }
    }

    for (auto vertex : intersection_points) {
      if (vertex->marked) {
        continue;
      }

      vertex->marked = true;

      std::vector<Point> pts;

      pts.emplace_back(vertex->point);

      auto curr = vertex;

      do {
        if (curr->entry_exit) {
          do {
            curr = curr->prev;

            pts.emplace_back(curr->point);
          } while (!curr->intersect);
        } else {
          do {
            curr = curr->next;

            pts.emplace_back(curr->point);
          } while (!curr->intersect);
        }
        curr->marked = true;
        curr = curr->neighbour;
        curr->marked = true;
      } while (curr != vertex);

      result.append_vertices(pts);
    }

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

PolygonClip ClipAlgorithm::do_diff(PolygonClip subject, PolygonClip clipping) {
  PolygonClip result;

  ClipAlgorithm algorithm(std::move(subject), std::move(clipping));

  algorithm.process_intersection();

  bool no_intersection;
  uint32_t inner_indicator;

  std::tie(no_intersection, inner_indicator) = algorithm.mark_vertices();

  if (!no_intersection) {
    std::vector<Vertex *> intersection_points;
    for (auto &vert : algorithm.m_subject.m_vertex) {
      if (vert->intersect) {
        intersection_points.emplace_back(vert.get());
      }
    }

    for (auto vertex : intersection_points) {
      if (vertex->marked) {
        continue;
      }

      vertex->marked = true;
      bool self = true;

      std::vector<Point> pts;

      auto curr = vertex;

      pts.emplace_back(curr->point);

      do {
        if (curr->entry_exit) {
          do {
            if (self) {
              curr = curr->prev;
            } else {
              curr = curr->next;
            }

            pts.emplace_back(curr->point);
          } while (!curr->intersect);
        } else {
          do {
            if (self) {
              curr = curr->next;
            } else {
              curr = curr->prev;
            }

            pts.emplace_back(curr->point);
          } while (!curr->intersect);
        }

        self = !self;
        curr->marked = true;
        curr = curr->neighbour;
        curr->marked = true;
      } while (curr != vertex);

      result.append_vertices(pts);
    }

    return result;
  }

  if (inner_indicator == 0) {
    // there is no common area between two polygons
    return PolygonClip(algorithm.m_subject);
  } else if (inner_indicator == 2) {
    // subject is inside clipping, no different part
    return result;
  }

  return PolygonClip(algorithm.m_subject, algorithm.m_clipping, true);
}

void ClipAlgorithm::process_intersection() {
  uint32_t intersection_count = 0;

  PolygonIter subj_iter(m_subject.get_vertices());

  while (subj_iter.has_next()) {
    auto current = subj_iter.current();

    PolygonIter clip_iter(m_clipping.get_vertices());

    std::vector<VertexDist> intersect_list;

    while (clip_iter.has_next()) {
      double t1 = 0.0;
      double t2 = 0.0;

      auto clip_curr = clip_iter.current();

      if (Math::segment_intersect(current, current->next, clip_curr,
                                  clip_curr->next, t1, t2)) {
        intersection_count++;

        auto i1 = m_subject.allocate_vertex(current, current->next, t1);
        auto i2 = m_clipping.allocate_vertex(clip_curr, clip_curr->next, t2);

        i1->intersect = true;
        i2->intersect = true;

        i1->neighbour = i2;
        i2->neighbour = i1;

        // insert i2 into cliping vertices list
        i2->next = clip_curr->next;
        clip_curr->next->prev = i2;

        clip_curr->next = i2;
        i2->prev = clip_curr;

        // insert i1 into intersect list
        intersect_list.emplace_back(VertexDist(i1, t1));

        clip_iter.move_next();
      }

      clip_iter.move_next();
    }

    if (!intersect_list.empty()) {

      std::sort(intersect_list.begin(), intersect_list.end(),
                VertDistCompiler{});

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

std::tuple<bool, uint32_t> ClipAlgorithm::mark_vertices() {
  bool no_intersection = true;
  uint32_t inner_indicator = 0;
  // false  : exit
  // true   : entry
  bool status = false;

  PolygonIter clip_iter(m_clipping.get_vertices());

  if (m_subject.contains(clip_iter.current()->point)) {
    status = false;
    inner_indicator = 1;
  } else {
    status = true;
    inner_indicator = 0;
  }

  while (clip_iter.has_next()) {
    auto current = clip_iter.current();

    if (current->intersect) {
      current->entry_exit = status;
      status = !status;

      if (no_intersection) {
        no_intersection = false;
      }
    }

    clip_iter.move_next();
  }

  // loop for PolygonClip 2
  PolygonIter subj_iter(m_subject.get_vertices());

  if (m_clipping.contains(subj_iter.current()->point)) {
    status = false;
    inner_indicator = 2;
  } else {
    status = true;
  }

  while (subj_iter.has_next()) {
    auto current = subj_iter.current();

    if (current->intersect) {
      current->entry_exit = status;
      status = !status;

      if (no_intersection) {
        no_intersection = false;
      }
    }

    subj_iter.move_next();
  }

  return std::make_tuple(no_intersection, inner_indicator);
}

PolygonClip::PolygonClip(const PolygonClip &other) {

  for (auto v : other.m_sub_polygons) {
    auto p = v;
    std::vector<Point> points{};
    points.emplace_back(p->point);

    p = p->next;

    while (p != v) {
      points.emplace_back(p->point);
      p = p->next;
    }

    if (points.size() < 3) {
      continue;
    }

    this->append_vertices(points);
  }
}

PolygonClip::PolygonClip(const PolygonClip &p1, const PolygonClip &p2, bool p2_reserve) {

  for (auto v : p1.m_sub_polygons) {
    auto p = v;
    std::vector<Point> points{};
    points.emplace_back(p->point);

    p = p->next;

    while (p != v) {
      points.emplace_back(p->point);
      p = p->next;
    }

    if (points.size() < 3) {
      continue;
    }

    this->append_vertices(points);
  }

  for (auto v : p2.m_sub_polygons) {
    auto p = v;
    std::vector<Point> points{};
    points.emplace_back(p->point);

    if (p2_reserve) {
      p = p->prev;
    } else {
      p = p->next;
    }

    while (p != v) {
      points.emplace_back(p->point);
      if (p2_reserve) {
        p = p->prev;
      } else {
        p = p->next;
      }
    }

    if (points.size() < 3) {
      continue;
    }

    this->append_vertices(points);
  }
}

void PolygonClip::append_vertices(const std::vector<Point> &points) {
  if (points.size() < 3) {
    // not a closed path
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

bool PolygonClip::contains(const Point &p) const {
    bool contains = false;
    int32_t winding_num = 0;

    PolygonIter iter(m_sub_polygons);
    double tolerance = 1e-6; // Tolerance for floating-point comparison

    while (iter.has_next()) {
        auto curr = iter.current();
        auto next = curr->next;

        // Check if the point is between the y coordinates of curr and next
        bool y_intersects = ((next->point.y() > p.y()) != (curr->point.y() > p.y())) &&
                            (p.x() < (curr->point.x() - next->point.x()) * (p.y() - next->point.y()) /
                               (curr->point.y() - next->point.y()) +
                               next->point.x());

        if (y_intersects) {
            contains = !contains;

            // Check if points are close to each other within tolerance
            if (std::abs(curr->point.x() - next->point.x()) < tolerance && 
                std::abs(curr->point.y() - next->point.y()) < tolerance) {
                iter.move_next();
                continue;
            }

            // Use tolerance to compare points
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

Vertex* PolygonClip::allocate_vertex(const Point& p) {
    if (!m_left_top) {
        m_left_top.emplace(p.pt);
    } else {
        if (m_left_top->x() >= p.x()) {
            m_left_top->pt = Point2d(p.pt.x(), m_left_top->pt.y());
        }
        if (m_left_top->y() >= p.y()) {
            m_left_top->pt = Point2d(m_left_top->pt.x(), p.pt.y());
        }
    }

    if (!m_right_bottom) {
        m_right_bottom.emplace(p.pt);
    } else {
        if (m_right_bottom->x() < p.x()) {
            m_right_bottom->pt = Point2d(p.pt.x(), m_right_bottom->pt.y());
        }
        if (m_right_bottom->y() < p.y()) {
            m_right_bottom->pt = Point2d(m_right_bottom->pt.x(), p.pt.y());
        }
    }

    // Allocate new Vertex with the point
    m_vertex.emplace_back(std::make_unique<Vertex>(p));

    return m_vertex.back().get();
}


Vertex* PolygonClip::allocate_vertex(Vertex* p1, Vertex* p2, double t) {
    Point p = (p1->point * (1.0 - t)) + (p2->point * t);
    return allocate_vertex(p);
}


PolygonClip PolygonClip::Clip(const PolygonClip &subject, const PolygonClip &clipping) {
  return ClipAlgorithm::do_clip(PolygonClip(subject), PolygonClip(clipping));
}

PolygonClip PolygonClip::Union(const PolygonClip &subject, const PolygonClip &clipping) {
  return ClipAlgorithm::do_union(PolygonClip(subject), PolygonClip(clipping));
}

PolygonClip PolygonClip::Diff(const PolygonClip &subject, const PolygonClip &clipping) {
  return ClipAlgorithm::do_diff(PolygonClip(subject), PolygonClip(clipping));
}

} // namespace pc