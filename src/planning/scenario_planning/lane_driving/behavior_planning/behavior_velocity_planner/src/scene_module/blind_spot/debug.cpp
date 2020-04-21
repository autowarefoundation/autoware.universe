/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <scene_module/blind_spot/scene.h>

#include "utilization/marker_helper.h"
#include "utilization/util.h"

namespace
{
using State = BlindSpotModule::State;

visualization_msgs::MarkerArray createDetectionAreaMarkerArray(
  const std::vector<geometry_msgs::Point> & detection_area, const State & state,
  const std::string & ns, const int64_t id)
{
  visualization_msgs::MarkerArray msg;

  if (detection_area.empty()) {
    return msg;
  }

  auto marker = createDefaultMarker(
    "map", ns.c_str(), 0, visualization_msgs::Marker::LINE_STRIP,
    createMarkerColor(0.0, 0.0, 0.0, 0.999));
  marker.lifetime = ros::Duration(0.3);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = createMarkerScale(0.1, 0.0, 0.0);

  marker.ns = ns;
  marker.id = id;  // to be unique

  if (state == State::STOP) {
    marker.color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
  } else {
    marker.color = createMarkerColor(0.0, 1.0, 1.0, 0.999);
  }

  // Add each vertex
  for (const auto & area_point : detection_area) {
    marker.points.push_back(area_point);
  }

  // Close polygon
  marker.points.push_back(marker.points.front());

  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_planning_msgs::PathWithLaneId & path, const std::string & ns, int64_t lane_id,
  double r, double g, double b)
{
  visualization_msgs::MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", ns.c_str(), lane_id, visualization_msgs::Marker::LINE_STRIP,
    createMarkerColor(r, g, b, 0.999));
  marker.lifetime = ros::Duration(0.3);
  marker.scale = createMarkerScale(0.3, 0.0, 0.0);

  for (const auto & p : path.points) {
    marker.points.push_back(p.point.pose.position);
  }

  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, const State & state, const std::string & ns, int64_t id,
  double r, double g, double b)
{
  visualization_msgs::MarkerArray msg;

  if (state == State::STOP) {
    auto marker_line = createDefaultMarker(
      "map", (ns + "_line").c_str(), 0, visualization_msgs::Marker::LINE_STRIP,
      createMarkerColor(r, g, b, 0.999));
    marker_line.id = id;
    marker_line.lifetime = ros::Duration(0.3);
    marker_line.scale = createMarkerScale(0.1, 0.0, 0.0);

    const double yaw = tf2::getYaw(pose.orientation);

    const double a = 3.0;
    geometry_msgs::Point p0;
    p0.x = pose.position.x - a * std::sin(yaw);
    p0.y = pose.position.y + a * std::cos(yaw);
    p0.z = pose.position.z;
    marker_line.points.push_back(p0);

    geometry_msgs::Point p1;
    p1.x = pose.position.x + a * std::sin(yaw);
    p1.y = pose.position.y - a * std::cos(yaw);
    p1.z = pose.position.z;
    marker_line.points.push_back(p1);

    msg.markers.push_back(marker_line);
  }

  return msg;
}

visualization_msgs::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::Pose & pose, int32_t lane_id)
{
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.header.stamp = ros::Time::now();
  marker_virtual_wall.ns = "stop_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = ros::Duration(0.5);
  marker_virtual_wall.type = visualization_msgs::Marker::CUBE;
  marker_virtual_wall.action = visualization_msgs::Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale.x = 0.1;
  marker_virtual_wall.scale.y = 5.0;
  marker_virtual_wall.scale.z = 2.0;
  marker_virtual_wall.color.r = 1.0;
  marker_virtual_wall.color.g = 0.0;
  marker_virtual_wall.color.b = 0.0;
  marker_virtual_wall.color.a = 0.5;
  msg.markers.push_back(marker_virtual_wall);

  visualization_msgs::Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.header.stamp = ros::Time::now();
  marker_factor_text.ns = "factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = ros::Duration(0.5);
  marker_factor_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = visualization_msgs::Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale.x = 0.0;
  marker_factor_text.scale.y = 0.0;
  marker_factor_text.scale.z = 1.0;
  marker_factor_text.color.r = 1.0;
  marker_factor_text.color.g = 1.0;
  marker_factor_text.color.b = 1.0;
  marker_factor_text.color.a = 0.999;
  marker_factor_text.text = "blind spot";
  msg.markers.push_back(marker_factor_text);

  return msg;
}

}  // namespace

visualization_msgs::MarkerArray BlindSpotModule::createDebugMarkerArray()
{
  visualization_msgs::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  appendMarkerArray(
    createPathMarkerArray(debug_data_.path_raw, "path_raw", lane_id_, 0.0, 1.0, 1.0),
    &debug_marker_array);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.stop_point_pose, state, "stop_point_pose", lane_id_, 1.0, 0.0, 0.0),
    &debug_marker_array);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.judge_point_pose, state, "judge_point_pose", lane_id_, 1.0, 1.0, 0.5),
    &debug_marker_array);

  appendMarkerArray(
    createPathMarkerArray(
      debug_data_.path_with_judgeline, "path_with_judgeline", lane_id_, 0.0, 0.5, 1.0),
    &debug_marker_array);

  appendMarkerArray(
    createDetectionAreaMarkerArray(
      debug_data_.detection_area, state, "blind_spot_detection_area", lane_id_),
    &debug_marker_array);

  appendMarkerArray(
    createPathMarkerArray(debug_data_.path_right_edge, "path_right_edge", lane_id_, 0.5, 0.0, 0.5),
    &debug_marker_array);

  appendMarkerArray(
    createPathMarkerArray(debug_data_.path_left_edge, "path_left_edge", lane_id_, 0.0, 0.5, 0.5),
    &debug_marker_array);

  if (state == BlindSpotModule::State::STOP) {
    appendMarkerArray(
      createVirtualWallMarkerArray(debug_data_.virtual_wall_pose, lane_id_), &debug_marker_array);
  }

  return debug_marker_array;
}
