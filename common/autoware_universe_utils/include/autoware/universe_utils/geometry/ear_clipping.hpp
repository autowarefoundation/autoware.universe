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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <utility>
#include <vector>

namespace autoware::universe_utils
{

using Polygon2d = autoware::universe_utils::Polygon2d;
using Point2d = autoware::universe_utils::Point2d;
using LinearRing2d = autoware::universe_utils::LinearRing2d;

struct Point {
    Point(std::size_t index, Point2d point) 
        : i(index), pt(std::move(point)), steiner(false), prev_index(0), next_index(0) {}

    std::size_t i;
    Point2d pt;
    bool steiner;
    std::size_t prev_index; // Use index for prev
    std::size_t next_index; // Use index for next

    [[nodiscard]] double x() const { return pt.x(); }
    [[nodiscard]] double y() const { return pt.y(); }
};

std::vector<Polygon2d> triangulate(const Polygon2d & polygon);

void perform_triangulation(const Polygon2d & polygon, std::vector<std::size_t> & indices);
std::size_t construct_point(std::size_t index, const Point2d & point, std::vector<Point> & points);
void remove_point(std::size_t index, std::vector<Point> & points);
bool is_ear(std::size_t ear_index, const std::vector<Point> & points);
bool sector_contains_sector(std::size_t m_index, std::size_t p_index, const std::vector<Point> & points);
bool point_in_triangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py);
bool is_valid_diagonal(std::size_t a_index, std::size_t b_index, const std::vector<Point> & points);
bool equals(std::size_t p1_index, std::size_t p2_index, const std::vector<Point> & points);
bool intersects(std::size_t p1_index, std::size_t q1_index, std::size_t p2_index, std::size_t q2_index, const std::vector<Point> & points);
bool on_segment(const std::vector<Point>& points, std::size_t pIdx, std::size_t qIdx, std::size_t rIdx);
bool intersects_polygon(const std::vector<Point>& points, std::size_t aIdx, std::size_t bIdx);
bool locally_inside(std::size_t a_index, std::size_t b_index, const std::vector<Point> & points);
bool middle_inside(std::size_t a_index, std::size_t b_index, const std::vector<Point> & points);
int sign(double val);
double area(const std::vector<Point>& points, std::size_t pIdx, std::size_t qIdx, std::size_t rIdx);

std::size_t linked_list(const LinearRing2d& points, bool clockwise, std::size_t& vertices, std::vector<Point> & points_vec);
std::size_t filter_points(std::size_t start_index, std::size_t end_index, std::vector<Point> & points_vec);
std::size_t cure_local_intersections(std::size_t start_index, std::vector<std::size_t> & indices, std::vector<Point> & points_vec);
std::size_t get_leftmost(std::size_t start_index, const std::vector<Point> & points);
std::size_t split_polygon(std::size_t a_index, std::size_t b_index, std::vector<Point> & points);
std::size_t insert_point(std::size_t i, const Point2d & p, std::vector<Point> & points, bool clockwise);
std::size_t eliminate_holes(const std::vector<LinearRing2d> & inners, std::size_t outer_index, std::size_t& vertices, std::vector<Point> & points);
std::size_t eliminate_hole(std::size_t hole_index, std::size_t outer_index, std::vector<Point> & points);
std::size_t find_hole_bridge(std::size_t hole_index, std::size_t outer_index, const std::vector<Point> & points);
void ear_clipping_linked(std::size_t ear_index, std::vector<std::size_t> & indices, std::vector<Point> & points, int pass = 0);
void split_ear_clipping(std::vector<Point>& points, std::size_t startIdx, std::vector<std::size_t>& indices);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__EAR_CLIPPING_HPP_
