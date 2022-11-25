// Copyright 2022 TIER IV, Inc.
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

#include "drivable_area_expander/debug.hpp"

#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace drivable_area_expander
{
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & ls, const double z)
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

visualization_msgs::msg::Marker makePolygonMarker(const polygon_t & polygon, const double z)
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
  const multipolygon_t & footprints, const multilinestring_t & uncrossable_lines,
  const multipolygon_t & predicted_paths, const double marker_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto line_id = 0lu;
  for (const auto & ls : uncrossable_lines) {
    auto marker = makeLinestringMarker(ls, marker_z);
    marker.ns = "uncrossable_lines";
    marker.id = line_id++;
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    debug_markers.markers.push_back(marker);
  }
  auto foot_id = 0lu;
  for (const auto & footprint : footprints) {
    auto marker = makePolygonMarker(footprint, marker_z);
    marker.color.g = 1.0;
    marker.color.a = 0.5;
    marker.id = foot_id++;
    marker.ns = "path_footprint";
    debug_markers.markers.push_back(marker);
  }
  auto pred_id = 0lu;
  for (const auto & poly : predicted_paths) {
    auto marker = makePolygonMarker(poly, marker_z);
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    marker.id = pred_id++;
    marker.ns = "predicted_paths";
    debug_markers.markers.push_back(marker);
  }

  static auto prev_max_line_id = 0lu;
  static auto prev_max_pred_id = 0lu;
  static auto prev_max_foot_id = 0lu;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = "uncrossable_lines";
  for (auto delete_id = line_id; delete_id < prev_max_line_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  marker.ns = "path_footprint";
  for (auto delete_id = pred_id; delete_id < prev_max_foot_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  marker.ns = "predicted_paths";
  for (auto delete_id = pred_id; delete_id < prev_max_pred_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  prev_max_line_id = line_id;
  prev_max_pred_id = pred_id;
  return debug_markers;
}
}  // namespace drivable_area_expander
