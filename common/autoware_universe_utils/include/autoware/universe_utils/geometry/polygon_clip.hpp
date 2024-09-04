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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware_universe::utils::pc
{

using Polygon2d = autoware::universe_utils::Polygon2d;
using Point2d = autoware::universe_utils::Point2d;
using LinearRing2d = autoware::universe_utils::LinearRing2d;

/**
 * @brief represents a 2D point with basic arithmetic operations.
 */
struct Point
{
  Point2d pt;

  Point() : pt(0.0, 0.0) {}
  explicit Point(const Point2d & point) : pt(point) {}
  explicit Point(double x, double y) : pt(x, y) {}

  double x() const { return pt.x(); }
  double y() const { return pt.y(); }
  void set_x(double x) { pt.x() = x; }
  void set_y(double y) { pt.y() = y; }

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

/**
 * @brief represents a vertex in a polygon with links to neighboring vertices.
 */
struct Vertex
{
  Point point;
  Vertex * prev = nullptr;
  Vertex * next = nullptr;
  bool intersect = false;
  bool entry_exit = false;
  bool marked = false;
  Vertex * neighbour = nullptr;

  Vertex() = default;
  explicit Vertex(Point point) : point(std::move(point)) {}
  Vertex(const Vertex &) = default;
  Vertex & operator=(const Vertex &) = default;
};

/**
 * @brief represents a polygon for clipping operations.
 */
class PolygonClip
{
  friend class ClipAlgorithm;

public:
  PolygonClip() = default;
  ~PolygonClip() = default;

  PolygonClip(const PolygonClip & other);
  PolygonClip(const PolygonClip & p1, const PolygonClip & p2, bool pr_reserve = false);
  explicit PolygonClip(const Polygon2d & input_polygon)
  {
    std::vector<Point> points;
    for (const auto & point : input_polygon.outer()) {
      points.emplace_back(point.x(), point.y());
    }
    append_vertices(points);
  };

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
  std::vector<Vertex *> m_sub_polygons;
  std::vector<std::unique_ptr<Vertex>> m_vertex;
  std::optional<Point> m_left_top;
  std::optional<Point> m_right_bottom;
};

/**
 * @brief enumeration for the types of polygon operations.
 */
enum class OperationType { CLIP, UNION, DIFF };

/**
 * @brief iterates over the vertices of a polygon.
 */
class PolygonIter
{
public:
  explicit PolygonIter(const std::vector<Vertex *> & polygons);
  ~PolygonIter() = default;

  bool has_next();
  void move_next();
  Vertex * current();

private:
  const std::vector<Vertex *> & m_polygon;
  size_t m_index = 0;
  bool m_loop_end = false;
  Vertex * m_curr_head = nullptr;
  Vertex * m_current = nullptr;
};

/**
 * @brief algorithm for performing clipping operations on polygons.
 */
class ClipAlgorithm
{
  enum class MarkType { kIntersection, kUnion, kDifference };

public:
  static PolygonClip process_operation(
    PolygonClip subject, PolygonClip clipping, OperationType op_type);

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

/**
 * @brief stores a vertex with its associated distance factor.
 */
struct VertexDist
{
  Vertex * vert;
  double t;

  VertexDist(Vertex * vert, double t) : vert(vert), t(t) {}
};

/**
 * @brief comparator for sorting VertexDist objects by distance factor.
 */
struct VertDistCompiler
{
  bool operator()(const VertexDist & v1, const VertexDist & v2) { return v1.t < v2.t; }
};

}  // namespace autoware_universe::utils::pc

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_
