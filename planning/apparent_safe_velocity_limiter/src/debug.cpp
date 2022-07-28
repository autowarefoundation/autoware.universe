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
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & ls, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.header.frame_id = "map";
  for (const auto & point : ls) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
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

visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const std::vector<Obstacle> & obstacles,
  const std::vector<multilinestring_t> & original_projections,
  const std::vector<multilinestring_t> & adjusted_projections,
  const std::vector<polygon_t> & original_footprints,
  const std::vector<polygon_t> & adjusted_footprints, const Float marker_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto id = 0;
  for (auto i = 0ul; i < original_projections.size(); ++i) {
    for (const auto ls : original_projections[i]) {
      auto marker = makeLinestringMarker(ls, marker_z);
      marker.ns = "original_projections";
      marker.id = id++;
      marker.color.r = 0.7;
      marker.color.b = 0.2;
      debug_markers.markers.push_back(marker);
    }
    for (const auto ls : adjusted_projections[i]) {
      auto marker = makeLinestringMarker(ls, marker_z);
      marker.ns = "adjusted_projections";
      marker.id = id++;
      marker.color.g = 0.7;
      marker.color.b = 0.2;
      debug_markers.markers.push_back(marker);
    }
    {
      auto marker = makePolygonMarker(original_footprints[i], marker_z);
      marker.ns = "original_footprints";
      marker.id = id++;
      marker.color.r = 0.7;
      debug_markers.markers.push_back(marker);
    }
    {
      auto marker = makePolygonMarker(adjusted_footprints[i], marker_z);
      marker.ns = "adjusted_footprints";
      marker.id = id++;
      marker.color.g = 0.7;
      debug_markers.markers.push_back(marker);
    }
  }
  auto obs_id = 0;
  for (const auto & obs : obstacles) {
    auto marker = makeLinestringMarker(obs.line, marker_z);
    marker.ns = "obstacles";
    marker.id = obs_id++;
    marker.color.b = 1.0;
    debug_markers.markers.push_back(marker);
  }

  static auto prev_max_id = 0lu;
  static auto prev_max_obs_id = 0lu;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = "obstacles";
  for (auto delete_id = obs_id; delete_id < prev_max_obs_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  for (const auto & ns :
       {"original_projections", "adjusted_projections", "original_footprints",
        "adjusted_footprints"}) {
    marker.ns = ns;
    for (auto delete_id = id; delete_id < prev_max_id; ++delete_id) {
      marker.id = delete_id;
      debug_markers.markers.push_back(marker);
    }
  }
  prev_max_id = id;
  prev_max_obs_id = obs_id;
  return debug_markers;
}

}  // namespace apparent_safe_velocity_limiter
