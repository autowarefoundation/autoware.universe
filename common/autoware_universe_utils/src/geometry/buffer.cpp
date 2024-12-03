// // Copyright 2024 TIER IV, Inc.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#include "autoware/universe_utils/geometry/buffer.hpp"
#include "autoware/universe_utils/geometry/temp_polygon_clip.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <iostream>
#include <vector>

namespace autoware::universe_utils
{

namespace offset_buffer
{

autoware::universe_utils::Polygon2d dissolve(autoware::universe_utils::Polygon2d const &polygon)
{
    autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
    auto extended_poly = autoware::universe_utils::polygon_clip::create_extended_polygon(polygon);
    sw.tic();
    double loop_1 = 0.0;
    double loop_2 = 0.0;
    autoware::universe_utils::polygon_clip::mark_self_intersections(extended_poly);
    loop_1 += sw.toc();
    sw.tic();

    auto polygon_vector = autoware::universe_utils::polygon_clip::construct_self_intersecting_polygons(extended_poly);
    std::cout << polygon_vector.outer().size() << "\n";
    loop_2 += sw.toc();

    std::cout << "time loop_1: " << loop_1 << "time loop_2: " << loop_2 << "\n";
    return polygon_vector;
}

autoware::universe_utils::Polygon2d create_arc(
  autoware::universe_utils::Polygon2d & vertices, const autoware::universe_utils::Point2d & center,
  double radius , const autoware::universe_utils::Point2d & end_vertex,
  const autoware::universe_utils::Point2d & start_vertex_next, double segments)
{
  const double PI2 = M_PI * 2;
  double start_angle = atan2(start_vertex_next.y() - center.y(), start_vertex_next.x() - center.x());
  double end_angle = atan2(end_vertex.y() - center.y(), end_vertex.x() - center.x());

  if (start_angle < 0) start_angle += PI2;
  if (end_angle < 0) end_angle += PI2;

  double angle_diff =
    ((start_angle > end_angle) ? (start_angle - end_angle) : (start_angle + PI2 - end_angle));
  if (angle_diff < 0) angle_diff += PI2;

  int dynamic_segments = static_cast<int>(segments * (angle_diff / PI2));
  if (dynamic_segments < 1) dynamic_segments = 1; 

  double segment_angle = angle_diff / dynamic_segments;

  for (int i = 0; i <= dynamic_segments; ++i) {
    double angle = end_angle + i * segment_angle;
    double x = center.x() + radius * cos(angle);
    double y = center.y() + radius * sin(angle);

    vertices.outer().push_back(autoware::universe_utils::Point2d(x, y));
  }


  return vertices;
}

void offset_segment(
  const autoware::universe_utils::Point2d &v1, const autoware::universe_utils::Point2d &v2,
  const autoware::universe_utils::Point2d &next_vertex, double dist, double segments, 
  autoware::universe_utils::Polygon2d &vertices)
{
  // Calculate direction and normals
  double dx = v2.x() - v1.x();
  double dy = v2.y() - v1.y();

  double length = std::sqrt(dx * dx + dy * dy);
  double normal_x = -dy / length;
  double normal_y = dx / length;

  autoware::universe_utils::Point2d offset_v1(v1.x() - normal_x * dist, v1.y() - normal_y * dist);
  autoware::universe_utils::Point2d offset_v2(v2.x() - normal_x * dist, v2.y() - normal_y * dist);


  double next_dx = next_vertex.x() - v2.x();
  double next_dy = next_vertex.y() - v2.y();
  double length_next = std::sqrt(next_dx * next_dx + next_dy * next_dy);
  double normal_x_next = -next_dy / length_next;
  double normal_y_next = next_dx / length_next;
  autoware::universe_utils::Point2d offset_v1next(
    v2.x() - normal_x_next * dist, v2.y() - normal_y_next * dist);
  autoware::universe_utils::Point2d offset_v2next(
    next_vertex.x() - normal_x_next * dist, next_vertex.y() - normal_y_next * dist);

  double current_angle = atan2(dy, dx);
  double next_angle = atan2(next_dy, next_dx);

  if (current_angle < 0) current_angle += M_PI * 2;
  if (next_angle < 0) next_angle += M_PI * 2;

  double angle_current_next = ((next_angle > current_angle) ? 
    (next_angle - current_angle) : 
    (next_angle + M_PI * 2 - current_angle));

  if (angle_current_next < 0) {
    angle_current_next += M_PI * 2;
  }

  if (angle_current_next < M_PI) {
    vertices.outer().push_back(offset_v1);
    create_arc(vertices, v2, dist,offset_v2, offset_v1next, segments);
  } else {
    vertices.outer().push_back(offset_v1);
    vertices.outer().push_back(offset_v2);
  } 
}


}

autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::Polygon2d & input_polygon, double dist, double segments)
{
  autoware::universe_utils::Polygon2d offset_polygon;
  autoware::universe_utils::Polygon2d final_polygon;
  size_t vertices_count = input_polygon.outer().size();

  if (vertices_count < 2) {
    std::cerr << "Polygon needs at least 2 vertices!" << std::endl;
    return offset_polygon;
  }

  std::vector<autoware::universe_utils::Point2d> new_ring(input_polygon.outer().begin(), input_polygon.outer().end() - 1);
  size_t modified_vertices_count = new_ring.size();
  
    for (size_t i = modified_vertices_count; i > 0; --i) {
      const auto &v1 = new_ring[(i - 1) % modified_vertices_count];
      const auto &v2 = new_ring[(i - 2 + modified_vertices_count) % modified_vertices_count];
      const auto &next_vertex = new_ring[(i - 3 + modified_vertices_count) % modified_vertices_count];

      autoware::universe_utils::offset_buffer::offset_segment(v1, v2, next_vertex, dist, segments, offset_polygon);
    }
  boost::geometry::correct(offset_polygon);
  auto result = autoware::universe_utils::offset_buffer::dissolve(offset_polygon);
  return result;
}

autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::Point2d & point, double distance, double segments)
{
  autoware::universe_utils::Polygon2d offset_polygon;

  for (int i = 0; i < segments; ++i) {
    double angle = 2 * M_PI * i / segments;
    autoware::universe_utils::Point2d buffer_point(
      point.x() + distance * cos(angle), point.y() + distance * sin(angle));
    offset_polygon.outer().push_back(buffer_point);
  }
  boost::geometry::correct(offset_polygon);
  return offset_polygon;
}

autoware::universe_utils::Polygon2d buffer(
  const autoware::universe_utils::MultiPoint2d & multi_point, double distance, double segments)
{
  autoware::universe_utils::Polygon2d buffer_union;
  bool first = true;
  for (const auto & point : multi_point) {
    autoware::universe_utils::Polygon2d buffer_point = buffer(point, distance, segments);
    if (!first) {
      buffer_union = autoware::universe_utils::union_(buffer_point, buffer_union)[0];
    } else {
      buffer_union = buffer_point;
      first = false;
    }
  }
  return buffer_union;
}

}  // namespace autoware::universe_utils
