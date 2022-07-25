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

#include <visualization_msgs/msg/marker_array.hpp>

namespace apparent_safe_velocity_limiter
{
visualization_msgs::msg::Marker makeLinestringMarker(const Obstacle & obstacle, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  for (const auto & point : obstacle.line) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

visualization_msgs::msg::MarkerArray makeLinestringMarkers(
  const std::vector<Obstacle> & obstacles, const Float z, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray markers;
  auto id = 0;
  for (const auto & obstacle : obstacles) {
    auto marker = makeLinestringMarker(obstacle, z);
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

visualization_msgs::msg::Marker makePolygonMarker(const polygon_t & polygon, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.frame_id = "map";
  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  geometry_msgs::msg::Point point;
  point.z = z;
  for (const auto & p : polygon.outer()) {
    point.x = p.x();
    point.y = p.y();
    marker.points.push_back(point);
  }
  return marker;
}

visualization_msgs::msg::Marker makePolygonPointsMarker(
  const std::vector<polygon_t> & polygons, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.header.frame_id = "map";
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.9;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  geometry_msgs::msg::Point point;
  point.z = z;
  for (const auto & polygon : polygons) {
    for (const auto & p : polygon.outer()) {
      point.x = p.x();
      point.y = p.y();
      marker.points.push_back(point);
    }
  }
  return marker;
}

visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const std::vector<Obstacle> & obstacles, const std::vector<polygon_t> & footprint_polygons,
  const polygon_t & envelope_polygon, const polygon_t & safe_envelope_polygon, const Float marker_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  // auto original_envelope = makeEnvelopeMarker(original_trajectory, projection_params);
  auto original_envelope = makePolygonMarker(envelope_polygon, marker_z);
  original_envelope.color.r = 1.0;
  original_envelope.ns = "original";
  debug_markers.markers.push_back(original_envelope);
  auto adjusted_envelope = makePolygonMarker(safe_envelope_polygon, marker_z);
  adjusted_envelope.color.g = 1.0;
  adjusted_envelope.ns = "adjusted";
  debug_markers.markers.push_back(adjusted_envelope);
  auto original_footprints = makePolygonPointsMarker(footprint_polygons, marker_z);
  original_footprints.ns = "original_footprints";
  debug_markers.markers.push_back(original_footprints);
  original_footprints.ns = "original_footprints";
  debug_markers.markers.push_back(original_footprints);

  static auto max_id = 0lu;
  const auto line_markers = makeLinestringMarkers(obstacles, marker_z, "obstacles");
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
