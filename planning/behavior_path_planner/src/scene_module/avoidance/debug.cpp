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

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace marker_utils::avoidance_marker
{
using behavior_path_planner::AvoidPoint;
using behavior_path_planner::util::shiftPose;
using visualization_msgs::msg::Marker;

MarkerArray createAvoidPointMarkerArray(
  const AvoidPointArray & shift_points, const std::string & ns, const float r, const float g,
  const float b, const double w)
{
  AvoidPointArray shift_points_local = shift_points;
  if (shift_points_local.empty()) {
    shift_points_local.push_back(AvoidPoint());
  }
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & sp : shift_points_local) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker = initializeMarker("map", ns, Marker::CUBE);
    marker.header.stamp = current_time;
    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.9);
    {
      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      // shiftPose(&marker_s.pose, current_shift);  // old
      shiftPose(&marker_s.pose, sp.start_length);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = Marker::LINE_STRIP;
      marker_l.scale = tier4_autoware_utils::createMarkerScale(w, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    // current_shift = sp.length;
  }

  return msg;
}

MarkerArray createAvoidanceObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, const std::string & ns)
{
  Marker marker = initializeMarker("map", ns, Marker::CUBE);
  marker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

  const auto normal_color = tier4_autoware_utils::createMarkerColor(0.9, 0.0, 0.0, 0.8);
  const auto disappearing_color = tier4_autoware_utils::createMarkerColor(0.9, 0.5, 0.9, 0.6);

  int32_t i = 0;
  MarkerArray msg;
  for (const auto & object : objects) {
    marker.id = i++;
    marker.action = Marker::ADD;
    marker.pose = object.object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(3.0, 1.5, 1.5);
    marker.color = object.lost_count == 0 ? normal_color : disappearing_color;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray makeOverhangToRoadShoulderMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects)
{
  Marker marker = initializeMarker("map", "overhang", Marker::TEXT_VIEW_FACING);
  marker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

  const auto normal_color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 1.0);

  int32_t i = 0;
  MarkerArray msg;
  for (const auto & object : objects) {
    marker.id = i++;
    // marker.action = Marker::ADD;
    marker.pose = object.overhang_pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(1.0, 1.0, 1.0);
    marker.color = normal_color;
    std::ostringstream string_stream;
    string_stream << "(to_road_shoulder_distance = " << object.to_road_shoulder_distance << " [m])";
    marker.text = string_stream.str();
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createOverhangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & linestring : linestrings) {
    Marker marker = initializeMarker("map", ns, linestring.id(), Marker::LINE_STRIP);
    marker.header.stamp = current_time;

    marker.action = Marker::ADD;
    marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = tier4_autoware_utils::createMarkerScale(0.4, 0.0, 0.0);
    marker.color = tier4_autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : linestring.basicLineString()) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
    marker.ns = "linestring id";
    marker.type = Marker::TEXT_VIEW_FACING;
    Pose text_id_pose;
    marker.scale = tier4_autoware_utils::createMarkerScale(1.5, 1.5, 1.5);
    marker.color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.8);
    text_id_pose.position.x = linestring.front().x();
    text_id_pose.position.y = linestring.front().y();
    text_id_pose.position.z = linestring.front().z();
    marker.pose = text_id_pose;
    std::ostringstream ss;
    ss << "(ID : " << linestring.id() << ") ";
    marker.text = ss.str();
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace marker_utils::avoidance_marker

std::string toStrInfo(const behavior_path_planner::ShiftPointArray & sp_arr)
{
  if (sp_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & sp : sp_arr) {
    ss << std::endl << toStrInfo(sp);
  }
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::ShiftPoint & sp)
{
  const auto & ps = sp.start.position;
  const auto & pe = sp.end.position;
  std::stringstream ss;
  ss << "shift length: " << sp.length << ", start_idx: " << sp.start_idx
     << ", end_idx: " << sp.end_idx << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x
     << ", " << pe.y << ")";
  return ss.str();
}

std::string toStrInfo(const behavior_path_planner::AvoidPointArray & ap_arr)
{
  if (ap_arr.empty()) {
    return "point is empty";
  }
  std::stringstream ss;
  for (const auto & ap : ap_arr) {
    ss << std::endl << toStrInfo(ap);
  }
  return ss.str();
}
std::string toStrInfo(const behavior_path_planner::AvoidPoint & ap)
{
  std::stringstream pids;
  for (const auto pid : ap.parent_ids) {
    pids << pid << ", ";
  }
  const auto & ps = ap.start.position;
  const auto & pe = ap.end.position;
  std::stringstream ss;
  ss << "id = " << ap.id << ", shift length: " << ap.length << ", start_idx: " << ap.start_idx
     << ", end_idx: " << ap.end_idx << ", start_dist = " << ap.start_longitudinal
     << ", end_dist = " << ap.end_longitudinal << ", start_length: " << ap.start_length
     << ", start: (" << ps.x << ", " << ps.y << "), end: (" << pe.x << ", " << pe.y
     << "), relative_length: " << ap.getRelativeLength() << ", grad = " << ap.getGradient()
     << ", parent_ids = [" << pids.str() << "]";
  return ss.str();
}
