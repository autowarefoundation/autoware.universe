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
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <vector>

namespace
{
std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

tier4_autoware_utils::Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & prev_pose, const geometry_msgs::msg::Pose & next_pose,
  const autoware_auto_perception_msgs::msg::Shape & shape)
{
  namespace bg = boost::geometry;
  const auto prev_poly = tier4_autoware_utils::toPolygon2d(prev_pose, shape);
  const auto next_poly = tier4_autoware_utils::toPolygon2d(next_pose, shape);

  tier4_autoware_utils::Polygon2d one_step_poly;
  for (const auto & point : prev_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }
  for (const auto & point : next_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }

  bg::correct(one_step_poly);

  tier4_autoware_utils::Polygon2d convex_one_step_poly;
  bg::convex_hull(one_step_poly, convex_one_step_poly);

  return convex_one_step_poly;
}

}  // namespace

namespace behavior_velocity_planner::intersection
{
namespace bg = boost::geometry;

ObjectInfo::ObjectInfo(const unique_identifier_msgs::msg::UUID & uuid) : uuid_str(::to_string(uuid))
{
}

void ObjectInfo::update(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  std::optional<lanelet::ConstLanelet> attention_lanelet_opt_,
  std::optional<lanelet::ConstLineString3d> stopline_opt_)
{
  predicted_object_ = object;
  attention_lanelet_opt = attention_lanelet_opt_;
  stopline_opt = stopline_opt_;
  if (attention_lanelet_opt) {
    const auto attention_lanelet = attention_lanelet_opt.value();
    observed_position_arc_coords = lanelet::utils::getArcCoordinates(
      {attention_lanelet}, predicted_object_.kinematics.initial_pose_with_covariance.pose);
  }
  observed_velocity = predicted_object_.kinematics.initial_twist_with_covariance.twist.linear.x;
  calc_dist_to_stopline();
}

void ObjectInfo::update(const std::optional<CollisionInterval> & unsafe_knowledge_opt)
{
  unsafe_decision_knowledge_ = unsafe_knowledge_opt;
}

void ObjectInfo::calc_dist_to_stopline()
{
  if (!stopline_opt || !attention_lanelet_opt) {
    return;
  }
  const auto attention_lanelet = attention_lanelet_opt.value();
  const auto object_arc_coords = lanelet::utils::getArcCoordinates(
    {attention_lanelet}, predicted_object_.kinematics.initial_pose_with_covariance.pose);
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

bool ObjectInfo::before_stopline_by(const double margin) const
{
  if (!dist_to_stopline_opt) {
    return false;
  }
  const double dist_to_stopline = dist_to_stopline_opt.value();
  return dist_to_stopline < margin;
}

std::shared_ptr<ObjectInfo> ObjectInfoManager::registerObject(
  const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area,
  const bool belong_intersection_area, const bool is_parked_vehicle)
{
  if (objects_info_.count(uuid) == 0) {
    auto object = std::make_shared<intersection::ObjectInfo>(uuid);
    objects_info_[uuid] = object;
  }
  auto object = objects_info_[uuid];
  if (belong_attention_area) {
    attention_area_objects_.push_back(object);
  } else if (belong_intersection_area) {
    intersection_area_objects_.push_back(object);
  }
  if (is_parked_vehicle) {
    parked_objects_.push_back(object);
  }
  return object;
}

void ObjectInfoManager::registerExistingObject(
  const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area,
  const bool belong_intersection_area, const bool is_parked_vehicle,
  std::shared_ptr<intersection::ObjectInfo> object)
{
  objects_info_[uuid] = object;
  if (belong_attention_area) {
    attention_area_objects_.push_back(object);
  } else if (belong_intersection_area) {
    intersection_area_objects_.push_back(object);
  }
  if (is_parked_vehicle) {
    parked_objects_.push_back(object);
  }
}

std::optional<intersection::CollisionInterval> findPassageInterval(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape,
  const lanelet::BasicPolygon2d & ego_lane_poly,
  const std::optional<lanelet::ConstLanelet> & first_attention_lane_opt,
  const std::optional<lanelet::ConstLanelet> & second_attention_lane_opt)
{
  const auto first_itr = std::adjacent_find(
    predicted_path.path.cbegin(), predicted_path.path.cend(), [&](const auto & a, const auto & b) {
      return bg::intersects(ego_lane_poly, ::createOneStepPolygon(a, b, shape));
    });
  if (first_itr == predicted_path.path.cend()) {
    // even the predicted path end does not collide with the beginning of ego_lane_poly
    return std::nullopt;
  }
  const auto last_itr = std::adjacent_find(
    predicted_path.path.crbegin(), predicted_path.path.crend(),
    [&](const auto & a, const auto & b) {
      return bg::intersects(ego_lane_poly, ::createOneStepPolygon(a, b, shape));
    });
  if (last_itr == predicted_path.path.crend()) {
    // even the predicted path start does not collide with the end of ego_lane_poly
    return std::nullopt;
  }

  const size_t enter_idx = static_cast<size_t>(first_itr - predicted_path.path.begin());
  const double object_enter_time =
    static_cast<double>(enter_idx) * rclcpp::Duration(predicted_path.time_step).seconds();
  const size_t exit_idx = static_cast<size_t>(last_itr.base() - predicted_path.path.begin());
  const double object_exit_time =
    static_cast<double>(exit_idx) * rclcpp::Duration(predicted_path.time_step).seconds();
  const auto [lane_position, lane_id] =
    [&]() -> std::pair<intersection::CollisionInterval::LanePosition, lanelet::Id> {
    if (second_attention_lane_opt) {
      if (lanelet::geometry::inside(
            second_attention_lane_opt.value(),
            lanelet::BasicPoint2d(first_itr->position.x, first_itr->position.y))) {
        return std::make_pair(
          intersection::CollisionInterval::LanePosition::SECOND,
          second_attention_lane_opt.value().id());
      }
    }
    if (first_attention_lane_opt) {
      if (lanelet::geometry::inside(
            first_attention_lane_opt.value(),
            lanelet::BasicPoint2d(first_itr->position.x, first_itr->position.y))) {
        return std::make_pair(
          intersection::CollisionInterval::LanePosition::FIRST,
          first_attention_lane_opt.value().id());
      }
    }
    return std::make_pair(intersection::CollisionInterval::LanePosition::ELSE, lanelet::InvalId);
  }();

  std::vector<geometry_msgs::msg::Pose> path;
  for (const auto & pose : predicted_path.path) {
    path.push_back(pose);
  }
  return intersection::CollisionInterval{
    lane_position, lane_id, path, {enter_idx, exit_idx}, {object_enter_time, object_exit_time}};
}

}  // namespace behavior_velocity_planner::intersection
