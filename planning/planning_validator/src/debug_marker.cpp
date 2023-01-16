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

#include "planning_validator/debug_marker.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <vector>

using visualization_msgs::msg::Marker;

PlanningValidatorDebugMarkerPublisher::PlanningValidatorDebugMarkerPublisher(rclcpp::Node * node)
: node_(node)
{
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);

  virtual_wall_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
}

void PlanningValidatorDebugMarkerPublisher::clearMarkers()
{
  marker_array_.markers.clear();
  marker_array_virtual_wall_.markers.clear();
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & p, const std::string & ns, int id)
{
  pushPoseMarker(p.pose, ns, id);
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, int id)
{
  // append arrow marker
  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = ns;
  marker.id = getMarkerId(ns);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.pose = pose;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.2, 0.1, 0.3);
  if (id == 0)  // Red
  {
    marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999);
  }
  if (id == 1)  // Green
  {
    marker.color = tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
  }
  if (id == 2)  // Blue
  {
    marker.color = tier4_autoware_utils::createMarkerColor(0.0, 0.0, 1.0, 0.999);
  }
  marker_array_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushWarningMsg(
  const geometry_msgs::msg::Pose & pose, const std::string & msg)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "warning_msg";
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.text = msg;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.0, 0.0, 1.0);
  marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.999);
  marker_array_virtual_wall_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushVirtualWall(const geometry_msgs::msg::Pose & pose)
{
  const auto now = node_->get_clock()->now();
  const auto stop_wall_marker =
    motion_utils::createStopVirtualWallMarker(pose, "planning_validator", now, 0);
  tier4_autoware_utils::appendMarkerArray(stop_wall_marker, &marker_array_virtual_wall_, now);
}

void PlanningValidatorDebugMarkerPublisher::publish()
{
  debug_viz_pub_->publish(marker_array_);
  virtual_wall_pub_->publish(marker_array_virtual_wall_);
}
