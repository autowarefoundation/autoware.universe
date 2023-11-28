// Copyright 2023 TIER IV, Inc.
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

#include "debug.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop::debug
{
namespace
{
// visualization_msgs::msg::Marker get_base_marker()
// {
//   visualization_msgs::msg::Marker base_marker;
//   base_marker.header.frame_id = "map";
//   base_marker.header.stamp = rclcpp::Time(0);
//   base_marker.id = 0;
//   base_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//   base_marker.action = visualization_msgs::msg::Marker::ADD;
//   base_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0);
//   base_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
//   base_marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1);
//   base_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
//   return base_marker;
// }
}  // namespace

std::vector<visualization_msgs::msg::Marker> make_delete_markers(
  const size_t from, const size_t to, const std::string & ns)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = ns;
  for (marker.id = static_cast<int>(from); marker.id < static_cast<int>(to); ++marker.id)
    markers.push_back(marker);
  return markers;
}

std::vector<visualization_msgs::msg::Marker> make_dynamic_obstacle_markers(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & obstacles)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = "dynamic_obstacles";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = tier4_autoware_utils::createMarkerScale(1.0, 1.0, 1.0);
  marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 1.0);
  marker.text = "dynamic obstacle";
  for (const auto & obstacle : obstacles) {
    marker.pose = obstacle.kinematics.initial_pose_with_covariance.pose;
    markers.push_back(marker);
    ++marker.id;
  }
  return markers;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop::debug
