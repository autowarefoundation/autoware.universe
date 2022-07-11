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

#include "apparent_safe_velocity_limiter/debug.hpp"

#include "apparent_safe_velocity_limiter/forward_projection.hpp"
#include "apparent_safe_velocity_limiter/types.hpp"

#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

namespace apparent_safe_velocity_limiter
{
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & lines, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  for (const auto & point : lines) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

visualization_msgs::msg::MarkerArray makeLinestringMarkers(
  const multilinestring_t & lines, const Float z, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray markers;
  auto id = 0;
  for (const auto & line : lines) {
    auto marker = makeLinestringMarker(line, z);
    marker.header.frame_id = "map";
    marker.id = id++;
    marker.ns = ns;
    markers.markers.push_back(marker);
  }
  return markers;
}

visualization_msgs::msg::Marker makeEnvelopeMarker(
  const Trajectory & trajectory, ProjectionParameters & projection_params)
{
  visualization_msgs::msg::Marker envelope;
  envelope.header = trajectory.header;
  envelope.type = visualization_msgs::msg::Marker::LINE_STRIP;
  envelope.scale.x = 0.1;
  envelope.color.a = 1.0;
  for (const auto & point : trajectory.points) {
    projection_params.update(point);
    const auto vector = forwardSimulatedSegment(point.pose.position, projection_params);
    geometry_msgs::msg::Point p;
    p.x = vector.second.x();
    p.y = vector.second.y();
    p.z = point.pose.position.z;
    envelope.points.push_back(p);
  }
  return envelope;
}

visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const Trajectory & original_trajectory, const Trajectory & adjusted_trajectory,
  const multilinestring_t & lines, ProjectionParameters & projection_params, const Float marker_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto original_envelope = makeEnvelopeMarker(original_trajectory, projection_params);
  original_envelope.color.r = 1.0;
  original_envelope.ns = "original";
  debug_markers.markers.push_back(original_envelope);
  auto adjusted_envelope = makeEnvelopeMarker(adjusted_trajectory, projection_params);
  adjusted_envelope.color.g = 1.0;
  adjusted_envelope.ns = "adjusted";
  debug_markers.markers.push_back(adjusted_envelope);

  static auto max_id = 0lu;
  const auto line_markers = makeLinestringMarkers(lines, marker_z, "obstacles");
  debug_markers.markers.insert(
    debug_markers.markers.begin(), line_markers.markers.begin(), line_markers.markers.end());
  auto id = line_markers.markers.size();
  max_id = std::max(id, max_id);
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = "obstacles";
  while (id <= max_id) {
    marker.id = id++;
    debug_markers.markers.push_back(marker);
  }
  return debug_markers;
}

}  // namespace apparent_safe_velocity_limiter
