// Copyright 2024 Tier IV, Inc.
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

#include "object_manager.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <vector>

namespace behavior_velocity_planner::intersection
{
namespace bg = boost::geometry;

void ObjectInfo::update(
  std::optional<lanelet::ConstLanelet> attention_lanelet_opt_,
  std::optional<lanelet::ConstLineString3d> stopline_opt_,
  const autoware_auto_perception_msgs::msg::PredictedObject & predicted_object,
  const std::optional<CollisionKnowledge> & unsafe_knowledge_opt)
{
  observed_position = predicted_object.kinematics.initial_pose_with_covariance.pose;
  attention_lanelet_opt = attention_lanelet_opt_;
  stopline_opt = stopline_opt_;
  if (attention_lanelet_opt) {
    const auto attention_lanelet = attention_lanelet_opt.value();
    observed_position_arc_coords =
      lanelet::utils::getArcCoordinates({attention_lanelet}, observed_position);
  }
  observed_velocity = predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;
  unsafe_decision_knowledge = unsafe_knowledge_opt;
  calc_dist_to_stopline();
}

void ObjectInfo::calc_dist_to_stopline()
{
  if (!stopline_opt || !attention_lanelet_opt) {
    return;
  }
  const auto attention_lanelet = attention_lanelet_opt.value();
  const auto object_arc_coords =
    lanelet::utils::getArcCoordinates({attention_lanelet}, observed_position);
  const auto stopline = stopline_opt.value();
  geometry_msgs::msg::Pose stopline_center;
  stopline_center.position.x = (stopline.front().x() + stopline.back().x()) / 2.0;
  stopline_center.position.y = (stopline.front().y() + stopline.back().y()) / 2.0;
  stopline_center.position.z = (stopline.front().z() + stopline.back().z()) / 2.0;
  const auto stopline_arc_coords =
    lanelet::utils::getArcCoordinates({attention_lanelet}, stopline_center);
  dist_to_stopline_opt = (stopline_arc_coords.length - object_arc_coords.length);
}

bool ObjectInfo::can_stop_before_stopline(const double brake_deceleration) const
{
  if (!dist_to_stopline_opt) {
    return false;
  }
  const double dist_to_stopline = dist_to_stopline_opt.value();
  const double braking_distance =
    (observed_velocity * observed_velocity) / (2.0 * brake_deceleration);
  return dist_to_stopline > braking_distance;
}

bool ObjectInfo::can_stop_before_ego_lane(
  const double brake_deceleration, const double tolerable_overshoot,
  lanelet::ConstLanelet ego_lane) const
{
  if (!dist_to_stopline_opt || !stopline_opt || !attention_lanelet_opt) {
    return false;
  }
  const double dist_to_stopline = dist_to_stopline_opt.value();
  const double braking_distance =
    (observed_velocity * observed_velocity) / (2.0 * brake_deceleration);
  if (dist_to_stopline > braking_distance) {
    return false;
  }
  const auto attention_lanelet = attention_lanelet_opt.value();
  const auto stopline = stopline_opt.value();
  const auto stopline_p1 = stopline.front();
  const auto stopline_p2 = stopline.back();
  const tier4_autoware_utils::Point2d stopline_mid{
    stopline_p1.x() + stopline_p2.x() / 2.0, (stopline_p1.y() + stopline_p2.y()) / 2.0};
  const auto attention_lane_end = attention_lanelet.centerline().back();
  const tier4_autoware_utils::LineString2d attention_lane_later_part(
    {tier4_autoware_utils::Point2d{stopline_mid.x(), stopline_mid.y()},
     tier4_autoware_utils::Point2d{attention_lane_end.x(), attention_lane_end.y()}});
  std::vector<tier4_autoware_utils::Point2d> ego_collision_points;
  bg::intersection(
    attention_lane_later_part, ego_lane.centerline2d().basicLineString(), ego_collision_points);
  if (ego_collision_points.empty()) {
    return false;
  }
  const auto expected_collision_point = ego_collision_points.front();
  // distance from object expected stop position to collision point
  const double stopline_to_object = -1.0 * dist_to_stopline + braking_distance;
  const double stopline_to_ego_path = std::hypot(
    expected_collision_point.x() - stopline_mid.x(),
    expected_collision_point.y() - stopline_mid.y());
  const double object_to_ego_path = stopline_to_ego_path - stopline_to_object;
  // NOTE: if object_to_ego_path < 0, object passed ego path
  return object_to_ego_path > tolerable_overshoot;
}
}  // namespace behavior_velocity_planner::intersection
