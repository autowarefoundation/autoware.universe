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

visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns,
  const int64_t lane_id)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = lanelet.id();
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (const auto & p : lanelet.polygon3d()) {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createPolygonMarkerArray(
  const lanelet::CompoundPolygon3d & polygon, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  int32_t uid = planning_utils::bitShift(lane_id);
  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;

  marker.ns = ns;
  marker.id = uid;
  marker.lifetime = ros::Duration(0.3);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = createMarkerScale(0.3, 0.0, 0.0);
  marker.color = createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon) {
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::DynamicObjectArray & objects, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t uid = planning_utils::bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.lifetime = ros::Duration(1.0);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.state.pose_covariance.pose;
    marker.scale = createMarkerScale(1.0, 1.0, 1.0);
    marker.color = createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_planning_msgs::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = ros::Duration(0.3);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = createMarkerScale(0.3, 0.0, 0.0);
  marker.color = createMarkerColor(r, g, b, 0.999);

  for (const auto & p : path.points) {
    marker.points.push_back(p.point.pose.position);
  }

  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::Pose & pose, const int64_t lane_id)
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
  marker_virtual_wall.scale = createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = createMarkerColor(1.0, 0.0, 0.0, 0.5);
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
  marker_factor_text.scale = createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = "blind spot";
  msg.markers.push_back(marker_factor_text);

  return msg;
}

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, const State & state, const std::string & ns, const int64_t id,
  const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  if (state == State::STOP) {
    visualization_msgs::Marker marker_line{};
    marker_line.header.frame_id = "map";
    marker_line.header.stamp = current_time;
    marker_line.ns = ns + "_line";
    marker_line.id = id;
    marker_line.lifetime = ros::Duration(0.3);
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker_line.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker_line.color = createMarkerColor(r, g, b, 0.999);

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
    createPolygonMarkerArray(
      debug_data_.confict_area_for_blind_spot, "conflict_area_for_blind_spot", lane_id_, 0.0, 0.5,
      0.5),
    &debug_marker_array);

  appendMarkerArray(
    createPolygonMarkerArray(
      debug_data_.detection_area_for_blind_spot, "detection_area_for_blind_spot", lane_id_, 0.0,
      0.5, 0.5),
    &debug_marker_array);

  appendMarkerArray(
    createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", lane_id_, 0.99, 0.4, 0.0),
    &debug_marker_array);

  appendMarkerArray(
    createPathMarkerArray(debug_data_.spline_path, "spline", lane_id_, 0.5, 0.5, 0.5),
    &debug_marker_array);

  if (state == BlindSpotModule::State::STOP) {
    appendMarkerArray(
      createVirtualWallMarkerArray(debug_data_.virtual_wall_pose, lane_id_), &debug_marker_array);
  }

  return debug_marker_array;
}
