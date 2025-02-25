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

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <string>
#include <vector>
namespace autoware::behavior_velocity_planner::debug
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;

visualization_msgs::msg::MarkerArray create_polygon_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const std::string & ns, const int64_t module_id,
  const rclcpp::Time & now, const double x, const double y, const double z, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;
  {
    auto marker = create_default_marker(
      "map", now, ns, static_cast<int32_t>(module_id), visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)),
      create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8f));
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    for (const auto & p : polygon.points) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      marker.points.push_back(point);
    }

    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray create_path_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & p = path.points.at(i);

    auto marker = create_default_marker(
      "map", now, ns, static_cast<int32_t>(planning_utils::bitShift(lane_id) + i),
      visualization_msgs::msg::Marker::ARROW,
      create_marker_scale(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)),
      create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f));
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.pose = p.point.pose;

    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // if p.lane_ids has lane_id
      marker.color = create_marker_color(
        static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f);
    } else {
      marker.color = create_marker_color(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray create_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    create_marker_color(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.8f));
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects.at(i);

    marker.id = static_cast<int>(planning_utils::bitShift(module_id) + i);
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray create_points_marker_array(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::SPHERE, create_marker_scale(x, y, z),
    create_marker_color(
      static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), 0.999f));
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  for (size_t i = 0; i < points.size(); ++i) {
    marker.id = static_cast<int32_t>(i + planning_utils::bitShift(module_id));
    marker.pose.position = points.at(i);
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace autoware::behavior_velocity_planner::debug
