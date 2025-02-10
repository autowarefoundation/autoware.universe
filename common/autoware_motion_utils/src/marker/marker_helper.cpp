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

#include "autoware/motion_utils/marker/marker_helper.hpp"

#include "autoware_utils/ros/marker_helper.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

using autoware_utils::create_default_marker;
using autoware_utils::create_deleted_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using visualization_msgs::msg::MarkerArray;

namespace
{

inline visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::msg::Pose & vehicle_front_pose, const std::string & module_name,
  const std::string & ns_prefix, const rclcpp::Time & now, const int32_t id,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Virtual Wall
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "virtual_wall", id, visualization_msgs::msg::Marker::CUBE,
      create_marker_scale(0.1, 5.0, 2.0), color);

    marker.pose = vehicle_front_pose;
    marker.pose.position.z += 1.0;

    marker_array.markers.push_back(marker);
  }

  // Factor Text
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "factor_text", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 1.0 /*font size*/), create_marker_color(1.0, 1.0, 1.0, 1.0));

    marker.pose = vehicle_front_pose;
    marker.pose.position.z += 2.0;
    marker.text = module_name;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

inline visualization_msgs::msg::MarkerArray createDeletedVirtualWallMarkerArray(
  const std::string & ns_prefix, const rclcpp::Time & now, const int32_t id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Virtual Wall
  {
    auto marker = create_deleted_default_marker(now, ns_prefix + "virtual_wall", id);
    marker_array.markers.push_back(marker);
  }

  // Factor Text
  {
    auto marker = create_deleted_default_marker(now, ns_prefix + "factor_text", id);
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

inline visualization_msgs::msg::MarkerArray createIntendedPassArrowMarkerArray(
  const geometry_msgs::msg::Pose & vehicle_front_pose, const std::string & module_name,
  const std::string & ns_prefix, const rclcpp::Time & now, const int32_t id,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Arrow
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "direction", id, visualization_msgs::msg::Marker::ARROW,
      create_marker_scale(2.5 /*length*/, 0.15 /*width*/, 1.0 /*height*/), color);

    marker.pose = vehicle_front_pose;

    marker_array.markers.push_back(marker);
  }

  // Factor Text
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "factor_text", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 0.4 /*font size*/), create_marker_color(1.0, 1.0, 1.0, 1.0));

    marker.pose = vehicle_front_pose;
    marker.pose.position.z += 2.0;
    marker.text = module_name;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace

namespace autoware::motion_utils
{
visualization_msgs::msg::MarkerArray createStopVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset, const std::string & ns_prefix,
  const bool is_driving_forward)
{
  const auto pose_with_offset = autoware_utils::calc_offset_pose(
    pose, longitudinal_offset * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, ns_prefix + "stop_", now, id,
    create_marker_color(1.0, 0.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createSlowDownVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset, const std::string & ns_prefix,
  const bool is_driving_forward)
{
  const auto pose_with_offset = autoware_utils::calc_offset_pose(
    pose, longitudinal_offset * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, ns_prefix + "slow_down_", now, id,
    create_marker_color(1.0, 1.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createDeadLineVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset, const std::string & ns_prefix,
  const bool is_driving_forward)
{
  const auto pose_with_offset = autoware_utils::calc_offset_pose(
    pose, longitudinal_offset * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, ns_prefix + "dead_line_", now, id,
    create_marker_color(0.0, 1.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createIntendedPassVirtualMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset, const std::string & ns_prefix,
  const bool is_driving_forward)
{
  const auto pose_with_offset = autoware_utils::calc_offset_pose(
    pose, longitudinal_offset * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  return createIntendedPassArrowMarkerArray(
    pose_with_offset, module_name, ns_prefix + "intended_pass_", now, id,
    create_marker_color(0.77, 0.77, 0.77, 0.5));
}

visualization_msgs::msg::MarkerArray createDeletedStopVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id)
{
  return createDeletedVirtualWallMarkerArray("stop_", now, id);
}

visualization_msgs::msg::MarkerArray createDeletedSlowDownVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id)
{
  return createDeletedVirtualWallMarkerArray("slow_down_", now, id);
}
}  // namespace autoware::motion_utils
