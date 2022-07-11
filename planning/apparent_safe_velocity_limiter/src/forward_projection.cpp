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

std::vector<segment_t> forwardSimulatedSegments(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params)
{
  const auto length = params.velocity * params.duration + params.extra_length;
  const auto from = point_t{origin.x, origin.y};
  const auto heading = params.heading;
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return {segment_t{from, to}};
}

polygon_t forwardSimulatedPolygon(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double lateral_offset)
{
  const auto length = params.velocity * params.duration + params.extra_length;
  const auto from = point_t{origin.x, origin.y};
  const auto heading = params.heading;
  const auto to =
    point_t{from.x() + std::cos(heading) * length, from.y() + std::sin(heading) * length};
  return generateFootprint(linestring_t{from, to}, lateral_offset);
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
