// Copyright 2024 Tier IV, Inc. All rights reserved.
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

#include "autoware/obstacle_collision_checker/debug.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>

namespace autoware::obstacle_collision_checker
{
visualization_msgs::msg::MarkerArray create_marker_array(
  const Output & output, const double base_link_z, const rclcpp::Time & now)
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;

  visualization_msgs::msg::MarkerArray marker_array;

  if (output.resampled_trajectory.points.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", now, "resampled_trajectory_line", 0, visualization_msgs::msg::Marker::LINE_STRIP,
        createMarkerScale(0.05, 0, 0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", now, "resampled_trajectory_points", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
        createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_collide = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output.will_collide) {
      color = color_will_collide;
    }

    auto marker = createDefaultMarker(
      "map", now, "vehicle_footprints", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto & p1 = vehicle_footprint.at(i);
        const auto & p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace autoware::obstacle_collision_checker
