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

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace autoware_universe::utils::pc
{

using Polygon2d = autoware::universe_utils::Polygon2d;
using Point2d = autoware::universe_utils::Point2d;
using LinearRing2d = autoware::universe_utils::LinearRing2d;

struct Point
{
  Point2d pt;

  // Default constructor
  Point() : pt(0.0, 0.0) {}

  // Constructor from Point2d
  Point(const Point2d & point) : pt(point) {}

  // Constructor from two doubles
  Point(double x, double y) : pt(x, y) {}

  // Accessor methods
  double x() const { return pt.x(); }
  double y() const { return pt.y(); }

  // Setter methods
  void set_x(double x) { pt.x() = x; }
  void set_y(double y) { pt.y() = y; }

  // Arithmetic operations
  Point operator*(double f) const { return Point(pt.x() * f, pt.y() * f); }

  Point operator+(const Point & other) const
  {
    return Point(pt.x() + other.pt.x(), pt.y() + other.pt.y());
  }

  Point operator-(const Point & other) const
  {
    return Point(pt.x() - other.pt.x(), pt.y() - other.pt.y());
  }
};

struct Vertex
{
  Point point = {};
  Vertex * prev = nullptr;
  Vertex * next = nullptr;
  bool intersect = false;
  bool entry_exit = false;
  bool marked = false;
  Vertex * neighbour = nullptr;

  Vertex() = default;

  Vertex(Point point) : point(std::move(point)) {}

  Vertex(const Vertex &) = default;
  Vertex & operator=(const Vertex &) = default;
};

class PolygonClip
{
  friend class ClipAlgorithm;

public:
  PolygonClip() = default;
  ~PolygonClip() = default;

  PolygonClip(const PolygonClip & other);
  PolygonClip(const PolygonClip & p1, const PolygonClip & p2, bool pr_reserve = false);

  // New constructor for Polygon2d input
  PolygonClip(const autoware::universe_utils::Polygon2d & input_polygon)
  {
    std::vector<Point> points;
    for (const auto & point : input_polygon.outer()) {
      points.emplace_back(point.x(), point.y());
    }
    append_vertices(points);
  }

  void append_vertices(const std::vector<Point> & points);

  const std::vector<Vertex *> & get_vertices() const { return m_sub_polygons; }

  bool contains(const Point & p) const;

  static PolygonClip Clip(const PolygonClip & subject, const PolygonClip & clipping);

  static PolygonClip Union(const PolygonClip & subject, const PolygonClip & clipping);

  static PolygonClip Diff(const PolygonClip & subject, const PolygonClip & clipping);

private:
  Vertex * allocate_vertex(const Point & p);

  Vertex * allocate_vertex(Vertex * p1, Vertex * p2, double t);

private:
  std::vector<Vertex *> m_sub_polygons = {};
  std::vector<std::unique_ptr<Vertex>> m_vertex = {};
  std::optional<Point> m_left_top = {};
  std::optional<Point> m_right_bottom = {};
};

////////////////////////////////////////////////////
class Math
{
public:
  static bool segment_intersect(
    Vertex * p1, Vertex * p2, Vertex * q1, Vertex * q2, double & t1, double & t2);
};

////////////////////////////////////////////////////

class PolygonIter
{
public:
  PolygonIter(const std::vector<Vertex *> & polygons);
  ~PolygonIter() = default;

  bool has_next();

  void move_next();

  Vertex * current();

private:
  size_t m_index;
  Vertex * m_curr_head;
  Vertex * m_current;
  bool m_loop_end = false;
  const std::vector<Vertex *> & m_polygon;
};

class ClipAlgorithm
{
  enum class MarkType {
    kIntersection,
    kUnion,
    kDifference,
  };

public:
  static PolygonClip do_clip(PolygonClip subject, PolygonClip clipping);
  static PolygonClip do_union(PolygonClip subject, PolygonClip clipping);
  static PolygonClip do_diff(PolygonClip subject, PolygonClip clipping);

private:
  ClipAlgorithm(PolygonClip subject, PolygonClip clipping)
  : m_subject(std::move(subject)), m_clipping(std::move(clipping))
  {
  }
  ~ClipAlgorithm() = default;

  void process_intersection();

  std::tuple<bool, uint32_t> mark_vertices();

private:
  PolygonClip m_subject;
  PolygonClip m_clipping;

  uint32_t m_intersect_count = 0;
};

}  // namespace autoware_universe::utils::pc
