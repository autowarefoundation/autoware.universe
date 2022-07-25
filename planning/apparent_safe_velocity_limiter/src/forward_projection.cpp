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
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/strategies/agnostic/buffer_distance_asymmetric.hpp>
#include <boost/geometry/strategies/agnostic/buffer_distance_symmetric.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/cartesian/buffer_end_flat.hpp>
#include <boost/geometry/strategies/cartesian/buffer_join_miter.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_circle.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

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

multilinestring_t bicycleProjectionLines(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  multilinestring_t lines;
  if (params.steering_angle_offset == 0.0)
    lines.push_back(bicycleProjectionLine(origin, params, params.steering_angle));
  else
    for (const auto offset : {params.steering_angle_offset, 0.0, -params.steering_angle_offset})
      lines.push_back(bicycleProjectionLine(origin, params, params.steering_angle + offset));
  return lines;
}

linestring_t bicycleProjectionLine(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double steering_angle)
{
  // TODO(Maxime CLEMENT): use Eigen for faster calculation
  linestring_t line;
  line.reserve(params.points_per_projection);
  line.emplace_back(origin.x, origin.y);
  const auto dt = params.duration / (params.points_per_projection - 1);
  const auto rotation_rate = params.velocity * std::tan(steering_angle) / params.wheel_base;
  for (auto i = 1; i < params.points_per_projection; ++i) {
    const auto t = i * dt;
    const auto heading = params.heading + rotation_rate * t;
    const auto length = params.velocity * t + params.extra_length;
    line.emplace_back(origin.x + length * std::cos(heading), origin.y + length * std::sin(heading));
  }
  return line;
}

polygon_t generateFootprint(const multilinestring_t & lines, const double lateral_offset)
{
  namespace bg = boost::geometry;
  polygon_t footprint;
  if (lines.size() == 1) {
    footprint = generateFootprint(lines.front(), lateral_offset);
  } else {  // assumes 3 lines ordered from left to right
    // calculate normal unit vector from the end a line to generate its left/right points
    constexpr auto perpendicular_point =
      [](const point_t & a, const point_t & b, const double offset) {
        const auto vector = (b - a).normalized();
        const auto normal_vector = point_t{-vector.y(), vector.x()};
        const auto point = b + (normal_vector * offset);
        return point_t{point.x(), point.y()};
      };
    footprint.outer().push_back(perpendicular_point(lines[0][1], lines[0][0], lateral_offset));
    for (auto it = lines[0].begin(); it != std::prev(lines[0].end()); ++it)
      footprint.outer().push_back(perpendicular_point(*it, *std::next(it), lateral_offset));
    // only use the left/right points at the end of the center line
    {
      footprint.outer().push_back(
        perpendicular_point(lines[1][lines[1].size() - 2], lines[1].back(), lateral_offset));
      footprint.outer().push_back(
        perpendicular_point(lines[1][lines[1].size() - 2], lines[1].back(), -lateral_offset));
    }
    // points to the right of the right line are added in reverse
    footprint.outer().push_back(
      perpendicular_point(lines[2][lines[2].size() - 2], lines[2].back(), -lateral_offset));
    for (auto it = lines[2].rbegin(); it != std::prev(lines[2].rend()); ++it)
      footprint.outer().push_back(perpendicular_point(*it, *std::next(it), lateral_offset));
  }
  footprint.outer().push_back(footprint.outer().front());  // close the polygon
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
  if (footprint.empty()) return {};
  return footprint[0];
}
}  // namespace apparent_safe_velocity_limiter
