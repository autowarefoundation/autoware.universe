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

#include <string>
#include <vector>

#include "tf2/utils.h"

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "behavior_path_planner/path_utilities.hpp"

namespace marker_utils
{
using visualization_msgs::msg::Marker;
using behavior_path_planner::util::shiftPose;
using behavior_path_planner::util::calcPathArcLengthArray;

inline int64_t bitShift(int64_t original_id) {return original_id << (sizeof(int32_t) * 8 / 2);}

MarkerArray createShiftPointMarkerArray(
  const std::vector<ShiftPoint> & shift_points, const double base_shift,
  const std::string & ns, const double r, const double g, const double b)
{
  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  double current_shift = base_shift;
  // TODO(Horibe) now assuming the shift point is aligned in longitudinal distance order
  for (const auto & sp : shift_points) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = rclcpp::Duration::from_seconds(1.1);
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.7);
    {
      marker.type = Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      shiftPose(&marker_s.pose, current_shift);
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
      marker_l.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    current_shift = sp.length;
  }

  return msg;
}

MarkerArray createFrenetPointMarkerArray(
  const std::vector<Frenet> & frenet_points,
  const PathWithLaneId & path, const Point & ego_point,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto length_back_to_ego = -autoware_utils::calcSignedArcLength(path.points, ego_point, 0);
  const auto arclength_arr = calcPathArcLengthArray(path);

  auto sorted_points = frenet_points;
  std::sort(
    sorted_points.begin(), sorted_points.end(), [](auto a, auto b) {
      return a.longitudinal < b.longitudinal;
    });

  int32_t id = 0;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;
  const auto addMarker = [&](const Pose & pose, const double shift) {
      auto shifted_pose = pose;
      shiftPose(&shifted_pose, shift);

      Marker marker{};
      marker.header.frame_id = "map";
      marker.header.stamp = current_time;
      marker.ns = ns;
      marker.id = id++;
      marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      marker.type = Marker::CUBE;
      marker.action = Marker::ADD;
      marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
      marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
      marker.color = autoware_utils::createMarkerColor(r, g, b, 0.7);
      marker.pose = shifted_pose;
      msg.markers.push_back(marker);
    };

  size_t frenet_points_idx = 0;
  for (size_t i = 0; i < arclength_arr.size(); ++i) {
    const double from_ego = arclength_arr.at(i) - length_back_to_ego;
    while (frenet_points_idx < sorted_points.size() &&
      sorted_points.at(frenet_points_idx).longitudinal < from_ego)
    {
      addMarker(path.points.at(i).point.pose, sorted_points.at(frenet_points_idx).lateral);
      ++frenet_points_idx;
    }
    if (frenet_points_idx == sorted_points.size()) {break;}
  }

  return msg;
}

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = lanelet.id();
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : lanelet.polygon3d()) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {marker.points.push_back(marker.points.front());}
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  int32_t i = 0;
  int32_t uid = bitShift(lane_id);
  for (const auto & polygon : polygons) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (const auto & p : polygon) {
      Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {marker.points.push_back(marker.points.front());}
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;

  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = autoware_utils::createMarkerScale(0.3, 0.0, 0.0);
  marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon.points) {
    Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {marker.points.push_back(marker.points.front());}
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray createObjectsMarkerArray(
  const DynamicObjectArray & objects, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.pose = object.state.pose_covariance.pose;
    marker.scale = autoware_utils::createMarkerScale(3.0, 1.0, 1.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;
  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & p : path.points) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = autoware_utils::createMarkerScale(0.6, 0.3, 0.3);
    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // if p.lane_ids has lane_id
      marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    } else {
      marker.color = autoware_utils::createMarkerColor(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createVirtualWallMarkerArray(
  const Pose & pose, const int64_t lane_id, const std::string & stop_factor)
{
  MarkerArray msg;

  Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  marker_virtual_wall.ns = "stop_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_virtual_wall.type = Marker::CUBE;
  marker_virtual_wall.action = Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale = autoware_utils::createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.5);
  msg.markers.push_back(marker_virtual_wall);

  Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  marker_factor_text.ns = "factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_factor_text.type = Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale = autoware_utils::createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = stop_factor;
  msg.markers.push_back(marker_factor_text);

  return msg;
}

MarkerArray createPoseMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  MarkerArray msg;

  Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.header.stamp = current_time;
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker_line.type = Marker::LINE_STRIP;
  marker_line.action = Marker::ADD;
  marker_line.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker_line.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker_line.color = autoware_utils::createMarkerColor(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

}  // namespace marker_utils
