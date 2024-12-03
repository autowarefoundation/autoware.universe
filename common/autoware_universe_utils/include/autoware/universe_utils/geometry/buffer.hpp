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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BUFFER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BUFFER_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware/universe_utils/geometry/polygon_clip.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

namespace autoware::universe_utils
{

namespace offset_buffer
{
/**
 * @brief create an arc between two vertices with a given radius and center
 * @param vertices the vertices to populate with the arc points
 * @param center the center point of the arc
 * @param radius the radius of the arc
 * @param start_vertex the start vertex of the arc
 * @param end_vertex the end vertex of the arc
 * @param start_vertex_next the next vertex after the start vertex
 * @param segments the number of segments to divide the arc into
 * @return the updated Polygon2d with the arc
 */
autoware::universe_utils::Polygon2d create_arc(
  autoware::universe_utils::Polygon2d & vertices, const autoware::universe_utils::Point2d & center,
  double radius, const autoware::universe_utils::Point2d & offset_v1, const autoware::universe_utils::Point2d & offset_v2next, const autoware::universe_utils::Point2d & end_vertex,
  const autoware::universe_utils::Point2d & start_vertex_next, double segments);

/**
 * @brief offset a polygon segment between two vertices with a specified distance
 * @param v1 the first vertex
 * @param v2 the second vertex
 * @param next_vertex the next vertex in the polygon
 * @param dist the offset distance
 * @param segments the number of segments to divide the arc into
 * @return the offset polygon segment
 */
void offset_segment(
  const autoware::universe_utils::Point2d & v1, const autoware::universe_utils::Point2d & v2,
  const autoware::universe_utils::Point2d & next_vertex, double dist, double segments, autoware::universe_utils::Polygon2d & vertices);
}  // namespace offset_buffer

/**
 * @brief buffer a polygon by a specified distance and number of segments
 * @param input_polygon the input polygon to be buffered
 * @param dist the offset distance
 * @param segments the number of segments to divide the arcs into
 * @return the buffered polygon
 */
autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::Polygon2d & input_polygon, double dist, double segments);

/**
 * @brief buffer a point by a specified distance and number of segments
 * @param point the point to be buffer
 * @param distance the offset distance
 * @param segments The number of segments to divide the arc into
 * @return the buffered polygon representing a circle (point buffer)
 */
autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::Point2d & point, double distance, double segments);

/**
 * @brief buffer (offset) multiple points and return the union of their buffers
 * @param multi_point a collection of points for the buffer
 * @param distance the offset distance
 * @param segments the number of segments to divide the arcs into
 * @return The union of all buffered polygons
 */
autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::MultiPoint2d & multi_point, double distance, double segments);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BUFFER_HPP_
