// Copyright 2022 Tier IV, Inc.
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

#include "apparent_safe_velocity_limiter/types.hpp"

#include <apparent_safe_velocity_limiter/forward_projection.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

namespace apparent_safe_velocity_limiter
{

segment_t forwardSimulatedSegment(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  const auto length = params.velocity * params.duration + params.extra_length;
  const auto from = point_t{origin.x, origin.y};
  const auto heading = params.heading;
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return segment_t{from, to};
}

std::vector<linestring_t> bicycleProjectionLines(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  std::vector<linestring_t> lines;
  const auto dt = params.duration / (params.points_per_projection - 1);
  point_t point;
  linestring_t line;
  // TODO(Maxime CLEMENT): use Eigen for faster calculation
  for (const auto steering_offset : params.steering_angle_offsets) {
    point.x(origin.x);
    point.y(origin.y);
    line = {point};
    const auto steering_angle = params.steering_angle + steering_offset;
    const auto rotation_rate = params.velocity * std::tan(steering_angle) / params.wheel_base;
    for (auto i = 1; i < params.points_per_projection; ++i) {
      const auto t = i * dt;
      const auto heading = params.heading + rotation_rate * t;
      const auto length = params.velocity * t + params.extra_length;
      point.x(origin.x + length * std::cos(heading));
      point.y(origin.y + length * std::sin(heading));
      line.push_back(point);
    }
    lines.push_back(line);
  }
  return lines;
}

polygon_t forwardSimulatedPolygon(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double lateral_offset, segment_t & projected_straight_segment)
{
  namespace bg = boost::geometry;
  polygon_t footprint;
  if (params.model == ProjectionParameters::PARTICLE) {
    projected_straight_segment = forwardSimulatedSegment(origin, params);
    footprint = generateFootprint(projected_straight_segment, lateral_offset);
  } else {  // ProjectionParameters::BICYCLE
    const auto lines = bicycleProjectionLines(origin, params);
    multipolygon_t union_polygons;
    multipolygon_t result_polygons;
    for (const auto & line : lines) {
      const auto line_footprint = generateFootprint(line, lateral_offset);
      bg::union_(line_footprint, union_polygons, result_polygons);
      union_polygons = result_polygons;
      bg::clear(result_polygons);
    }
    footprint = union_polygons.front();
    const auto straight_line = lines.size() == 1 ? lines[0] : lines[1];
    projected_straight_segment.first = straight_line.front();
    projected_straight_segment.second = straight_line.back();
  }
  return footprint;
}

polygon_t generateFootprint(const segment_t & segment, const double lateral_offset)
{
  return generateFootprint(linestring_t{segment.first, segment.second}, lateral_offset);
}

polygon_t generateFootprint(const linestring_t & linestring, const double lateral_offset)
{
  namespace bg = boost::geometry;
  multipolygon_t footprint;
  namespace strategy = bg::strategy::buffer;
  bg::buffer(
    linestring, footprint, strategy::distance_symmetric<double>(lateral_offset),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());
  return footprint[0];
}
}  // namespace apparent_safe_velocity_limiter
