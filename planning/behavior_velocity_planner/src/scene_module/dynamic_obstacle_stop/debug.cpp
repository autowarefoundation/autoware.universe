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

#include "scene_module/dynamic_obstacle_stop/debug.hpp"

#include "scene_module/dynamic_obstacle_stop/scene.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

namespace behavior_velocity_planner
{
namespace
{
visualization_msgs::msg::Marker createStopLineMarker(
  const geometry_msgs::msg::Pose & pose, const builtin_interfaces::msg::Time & current_time)
{
  constexpr int32_t marker_id = 0;
  auto marker = createDefaultMarker(
    "map", current_time, "stop_line", marker_id, visualization_msgs::msg::Marker::LINE_STRIP,
    createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 0, 0, 0.999));

  constexpr float line_length = 7.0;
  const auto p1 = tier4_autoware_utils::calcOffsetPose(pose, 0, line_length / 2.0, 0);
  const auto p2 = tier4_autoware_utils::calcOffsetPose(pose, 0, -line_length / 2.0, 0);
  marker.points.push_back(p1.position);
  marker.points.push_back(p2.position);

  return marker;
}

DynamicObstacleStopDebug::TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  const auto offset_pose = tier4_autoware_utils::calcOffsetPose(pose, 0, lateral_offset, 0);

  DynamicObstacleStopDebug::TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = offset_pose.position;

  return text_with_position;
}

DynamicObstacleStopDebug::TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Point position)
{
  DynamicObstacleStopDebug::TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = position;

  return text_with_position;
}

}  // namespace

DynamicObstacleStopDebug::DynamicObstacleStopDebug(rclcpp::Node & node) : node_(node)
{
  pub_debug_values_ = node.create_publisher<Float32MultiArrayStamped>(
    "~/dynamic_obstacle_stop/debug/debug_values", 1);
  pub_accel_reason_ =
    node.create_publisher<Int32Stamped>("~/dynamic_obstacle_stop/debug/accel_reason", 1);
  pub_debug_trajectory_ =
    node.create_publisher<Trajectory>("~/dynamic_obstacle_stop/debug/trajectory", 1);
}

void DynamicObstacleStopDebug::pushDebugPoints(const pcl::PointXYZ & debug_point)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = debug_point.x;
  ros_point.y = debug_point.y;
  ros_point.z = debug_point.z;

  debug_points_.push_back(ros_point);
}

void DynamicObstacleStopDebug::pushDebugPoints(const geometry_msgs::msg::Point & debug_point)
{
  debug_points_.push_back(debug_point);
}

void DynamicObstacleStopDebug::pushDebugPoints(
  const std::vector<geometry_msgs::msg::Point> & debug_points)
{
  for (const auto & p : debug_points) {
    debug_points_.push_back(p);
  }
}

void DynamicObstacleStopDebug::pushDebugPoints(
  const geometry_msgs::msg::Point & debug_point, const PointType point_type)
{
  switch (point_type) {
    case PointType::Blue:
      debug_points_.push_back(debug_point);
      break;

    case PointType::Red:
      debug_points_red_.push_back(debug_point);
      break;

    case PointType::Yellow:
      debug_points_yellow_.push_back(debug_point);
      break;

    default:
      break;
  }
}

void DynamicObstacleStopDebug::pushStopPose(const geometry_msgs::msg::Pose & pose)
{
  stop_pose_.emplace(pose);
}

void DynamicObstacleStopDebug::pushDebugLines(
  const std::vector<geometry_msgs::msg::Point> & debug_line)
{
  debug_lines_.push_back(debug_line);
}

void DynamicObstacleStopDebug::pushDebugPolygons(
  const std::vector<geometry_msgs::msg::Point> & debug_polygon)
{
  debug_polygons_.push_back(debug_polygon);
}

void DynamicObstacleStopDebug::pushDebugTexts(const TextWithPosition & debug_text)
{
  debug_texts_.push_back(debug_text);
}

void DynamicObstacleStopDebug::pushDebugTexts(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  debug_texts_.push_back(createDebugText(text, pose, lateral_offset));
}

void DynamicObstacleStopDebug::pushDebugTexts(
  const std::string text, const geometry_msgs::msg::Point position)
{
  debug_texts_.push_back(createDebugText(text, position));
}

void DynamicObstacleStopDebug::clearDebugMarker()
{
  debug_points_.clear();
  debug_points_red_.clear();
  debug_points_yellow_.clear();
  debug_lines_.clear();
  debug_polygons_.clear();
  debug_texts_.clear();
  stop_pose_.reset();
}

visualization_msgs::msg::MarkerArray DynamicObstacleStopDebug::createVisualizationMarkerArray()
{
  rclcpp::Time current_time = node_.now();

  // create marker array from debug data
  auto visualization_marker_array = createVisualizationMarkerArrayFromDebugData(current_time);

  if (stop_pose_) {
    visualization_marker_array.markers.emplace_back(
      createStopLineMarker(stop_pose_.value(), current_time));
  }

  // clear debug data
  clearDebugMarker();

  return visualization_marker_array;
}

visualization_msgs::msg::MarkerArray
DynamicObstacleStopDebug::createVisualizationMarkerArrayFromDebugData(
  const builtin_interfaces::msg::Time & current_time)
{
  visualization_msgs::msg::MarkerArray msg;

  if (!debug_points_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_points_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_points_red_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points_red", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 0, 0, 0.999));
    for (const auto & p : debug_points_red_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_points_yellow_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points_yellow", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(0.7, 0.7, 0.7), createMarkerColor(1.0, 1.0, 0, 0.999));
    for (const auto & p : debug_points_yellow_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_lines_.empty()) {
    int32_t marker_id = 0;
    for (const auto & line : debug_lines_) {
      auto marker = createDefaultMarker(
        "map", current_time, "debug_lines", marker_id, visualization_msgs::msg::Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(0, 0, 1.0, 0.999));
      for (const auto & p : line) {
        marker.points.push_back(p);
      }
      msg.markers.push_back(marker);
      marker_id++;
    }
  }

  if (!debug_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (const auto & poly : debug_polygons_) {
      for (size_t i = 0; i < poly.size() - 1; i++) {
        marker.points.push_back(poly.at(i));
        marker.points.push_back(poly.at(i + 1));
      }
      marker.points.push_back(poly.back());
      marker.points.push_back(poly.front());
    }

    msg.markers.push_back(marker);
  }

  if (!debug_texts_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_texts", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 0.8), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    constexpr float height_offset = 2.0;
    for (const auto & text : debug_texts_) {
      marker.pose.position = text.position;
      marker.pose.position.z += height_offset;
      marker.text = text.text;

      msg.markers.push_back(marker);
      marker.id++;
    }
  }

  return msg;
}
void DynamicObstacleStopDebug::setAccelReason(const AccelReason & accel_reason)
{
  accel_reason_ = accel_reason;
}

void DynamicObstacleStopDebug::publish()
{
  publishDebugValue();

  Int32Stamped accel_reason;
  accel_reason.stamp = node_.now();
  accel_reason.data = static_cast<int>(accel_reason_);
  pub_accel_reason_->publish(accel_reason);
}

void DynamicObstacleStopDebug::publishDebugValue()
{
  // publish debug values
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = node_.now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_values_->publish(debug_msg);
}

void DynamicObstacleStopDebug::publishDebugTrajectory(const Trajectory & trajectory)
{
  pub_debug_trajectory_->publish(trajectory);
}

// scene module
visualization_msgs::msg::MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  return debug_ptr_->createVisualizationMarkerArray();
}

}  // namespace behavior_velocity_planner
