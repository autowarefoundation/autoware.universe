// Copyright 2020 Tier IV, Inc.
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

#include <scene_module/speed_bump/scene.hpp>
#include <utilization/marker_helper.hpp>
#include <utilization/util.hpp>

#include <vector>

namespace behavior_velocity_planner
{
using motion_utils::createSlowDownVirtualWallMarker;
using DebugData = speed_bump_util::DebugData;

namespace
{
visualization_msgs::msg::MarkerArray createMarkers(
  const DebugData & debug_data, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(debug_data.base_link2front, 0.0, 0.0));

  int32_t uid = planning_utils::bitShift(module_id);

  // Collision line
  for (size_t i = 0; i < debug_data.collision_lines.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "collision line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.collision_lines.at(i).size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.collision_lines.at(i).at(j).x();
      point.y = debug_data.collision_lines.at(i).at(j).y();
      point.z = debug_data.collision_lines.at(i).at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision points
  if (!debug_data.collision_points.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "collision point";
    marker.id = uid;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.collision_points.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.collision_points.at(j).x();
      point.y = debug_data.collision_points.at(j).y();
      point.z = debug_data.collision_points.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Slow polygon
  for (size_t i = 0; i < debug_data.slow_polygons.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = debug_data.slow_polygons.at(i);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";

    marker.ns = "slow polygon line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!debug_data.slow_poses.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "slow point";
    marker.id = uid;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.slow_poses.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.slow_poses.at(j).position.x;
      point.y = debug_data.slow_poses.at(j).position.y;
      point.z = debug_data.slow_poses.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }
  // Acceleration point
  if (!debug_data.acc_poses.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "acceleration point";
    marker.id = uid;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.acc_poses.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.acc_poses.at(j).position.x;
      point.y = debug_data.acc_poses.at(j).position.y;
      point.z = debug_data.acc_poses.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace
visualization_msgs::msg::MarkerArray SpeedBumpModule::createVirtualWallMarkerArray()
{
  const auto now = this->clock_->now();
  auto id = module_id_;

  visualization_msgs::msg::MarkerArray wall_marker;

  for (const auto & p : debug_data_.slow_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      createSlowDownVirtualWallMarker(p_front, "speed_bump", now, id++), now, &wall_marker);
  }

  return wall_marker;
}
visualization_msgs::msg::MarkerArray SpeedBumpModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createMarkers(debug_data_, module_id_), this->clock_->now(), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace behavior_velocity_planner
