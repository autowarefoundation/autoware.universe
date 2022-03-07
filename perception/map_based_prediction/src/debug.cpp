// Copyright 2021 Tier IV, Inc.
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

#include "map_based_prediction_node.hpp"

namespace map_based_prediction
{
inline geometry_msgs::msg::Vector3 createMarkerScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::msg::ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

visualization_msgs::msg::Marker MapBasedPredictionNode::getDebugMarker(
  const TrackedObject & object, const Maneuver & maneuver,
  const size_t obj_num)
{
  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.ns = "maneuver";

  marker.id = static_cast<int32_t>(obj_num);
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = object.kinematics.pose_with_covariance.pose;
  marker.scale = createMarkerScale(3.0, 1.0, 1.0);

  // Color by maneuver
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  if (maneuver == Maneuver::LEFT_LANE_CHANGE) {
    g = 1.0;
  } else if (maneuver == Maneuver::RIGHT_LANE_CHANGE) {
    r = 1.0;
  } else {
    b = 1.0;
  }
  marker.color = createMarkerColor(r, g, b, 0.8);

  return marker;
}
} // map_based_prediction
