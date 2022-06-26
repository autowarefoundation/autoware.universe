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

namespace apparent_safe_velocity_limiter
{
visualization_msgs::msg::Marker makePolygonMarker(const linestring_t & polygon, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.ns = "obstacles";
  for (const auto & point : polygon) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

visualization_msgs::msg::Marker makeEnvelopeMarker(
  const Trajectory & trajectory, const ForwardProjectionFunction & fn)
{
  visualization_msgs::msg::Marker envelope;
  envelope.header = trajectory.header;
  envelope.type = visualization_msgs::msg::Marker::LINE_STRIP;
  envelope.scale.x = 0.1;
  envelope.color.a = 1.0;
  for (const auto & point : trajectory.points) {
    const auto vector = fn(point);
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
  const multilinestring_t & polygons, const ForwardProjectionFunction & fwd_sim_fn,
  const Float polygon_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto original_envelope = makeEnvelopeMarker(original_trajectory, fwd_sim_fn);
  original_envelope.color.r = 1.0;
  original_envelope.ns = "original";
  debug_markers.markers.push_back(original_envelope);
  auto adjusted_envelope = makeEnvelopeMarker(adjusted_trajectory, fwd_sim_fn);
  adjusted_envelope.color.g = 1.0;
  adjusted_envelope.ns = "adjusted";
  debug_markers.markers.push_back(adjusted_envelope);

  static auto max_id = 0;
  auto id = 0;
  for (const auto & poly : polygons) {
    auto marker = makePolygonMarker(poly, polygon_z);
    marker.header.frame_id = "map";
    marker.id = id++;
    debug_markers.markers.push_back(marker);
  }
  max_id = std::max(id, max_id);
  while (id <= max_id) {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.ns = "obstacles";
    marker.id = id++;
    debug_markers.markers.push_back(marker);
  }
  return debug_markers;
}

}  // namespace apparent_safe_velocity_limiter
