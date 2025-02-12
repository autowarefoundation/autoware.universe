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

#include "debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <string>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <memory>

namespace autoware::surround_obstacle_checker
{
namespace
{
Polygon2d createSelfPolygon(
  const VehicleInfo & vehicle_info, const double front_margin = 0.0, const double side_margin = 0.0,
  const double rear_margin = 0.0)
{
  const double & front_m = vehicle_info.max_longitudinal_offset_m + front_margin;
  const double & width_left_m = vehicle_info.max_lateral_offset_m + side_margin;
  const double & width_right_m = vehicle_info.min_lateral_offset_m - side_margin;
  const double & rear_m = vehicle_info.min_longitudinal_offset_m - rear_margin;

  Polygon2d ego_polygon;

  ego_polygon.outer().push_back(Point2d(front_m, width_left_m));
  ego_polygon.outer().push_back(Point2d(front_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_left_m));

  bg::correct(ego_polygon);

  return ego_polygon;
}
}  // namespace

using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;

SurroundObstacleCheckerDebugNode::SurroundObstacleCheckerDebugNode(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & object_label,
  const double & surround_check_front_distance, const double & surround_check_side_distance,
  const double & surround_check_back_distance, const double & surround_check_hysteresis_distance,
  const geometry_msgs::msg::Pose & self_pose, const rclcpp::Clock::SharedPtr clock,
  rclcpp::Node & node)
: planning_factor_interface_{std::make_unique<
    autoware::planning_factor_interface::PlanningFactorInterface>(
    &node, "surround_obstacle_checker")},
  vehicle_info_(vehicle_info),
  object_label_(object_label),
  surround_check_front_distance_(surround_check_front_distance),
  surround_check_side_distance_(surround_check_side_distance),
  surround_check_back_distance_(surround_check_back_distance),
  surround_check_hysteresis_distance_(surround_check_hysteresis_distance),
  self_pose_(self_pose),
  clock_(clock)
{
  debug_viz_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
  vehicle_footprint_pub_ = node.create_publisher<PolygonStamped>("~/debug/footprint", 1);
  vehicle_footprint_offset_pub_ =
    node.create_publisher<PolygonStamped>("~/debug/footprint_offset", 1);
  vehicle_footprint_recover_offset_pub_ =
    node.create_publisher<PolygonStamped>("~/debug/footprint_recover_offset", 1);
}

bool SurroundObstacleCheckerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::NoStart:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool SurroundObstacleCheckerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::NoStart:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

void SurroundObstacleCheckerDebugNode::publishFootprints()
{
  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  /* publish vehicle footprint polygon */
  const auto footprint = boostPolygonToPolygonStamped(ego_polygon, self_pose_.position.z);
  vehicle_footprint_pub_->publish(footprint);

  /* publish vehicle footprint polygon with offset */
  const auto polygon_with_offset = createSelfPolygon(
    vehicle_info_, surround_check_front_distance_, surround_check_side_distance_,
    surround_check_back_distance_);
  const auto footprint_with_offset =
    boostPolygonToPolygonStamped(polygon_with_offset, self_pose_.position.z);
  vehicle_footprint_offset_pub_->publish(footprint_with_offset);

  /* publish vehicle footprint polygon with recover offset */
  const auto polygon_with_recover_offset = createSelfPolygon(
    vehicle_info_, surround_check_front_distance_ + surround_check_hysteresis_distance_,
    surround_check_side_distance_ + surround_check_hysteresis_distance_,
    surround_check_back_distance_ + surround_check_hysteresis_distance_);
  const auto footprint_with_recover_offset =
    boostPolygonToPolygonStamped(polygon_with_recover_offset, self_pose_.position.z);
  vehicle_footprint_recover_offset_pub_->publish(footprint_with_recover_offset);
}

void SurroundObstacleCheckerDebugNode::publish()
{
  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* publish stop reason for autoware api */
  if (stop_pose_ptr_ != nullptr) {
    planning_factor_interface_->add(
      0.0, *stop_pose_ptr_, tier4_planning_msgs::msg::PlanningFactor::STOP,
      tier4_planning_msgs::msg::SafetyFactorArray{});
  }
  planning_factor_interface_->publish();

  /* reset variables */
  stop_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
}

MarkerArray SurroundObstacleCheckerDebugNode::makeVisualizationMarker()
{
  MarkerArray msg;
  rclcpp::Time current_time = this->clock_->now();

  // visualize surround object
  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = createDefaultMarker(
      "map", current_time, "no_start_obstacle_text", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;  // add half of the heights of obj roughly
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  return msg;
}

PolygonStamped SurroundObstacleCheckerDebugNode::boostPolygonToPolygonStamped(
  const Polygon2d & boost_polygon, const double & z)
{
  PolygonStamped polygon_stamped;
  polygon_stamped.header.frame_id = "base_link";
  polygon_stamped.header.stamp = this->clock_->now();

  for (auto const & p : boost_polygon.outer()) {
    geometry_msgs::msg::Point32 gp;
    gp.x = p.x();
    gp.y = p.y();
    gp.z = z;
    polygon_stamped.polygon.points.push_back(gp);
  }

  return polygon_stamped;
}

}  // namespace autoware::surround_obstacle_checker
