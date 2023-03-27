// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_
#define SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/utils.h>

#include <vector>

namespace behavior_velocity_planner
{
namespace out_of_lane_utils
{
inline tier4_autoware_utils::Polygon2d make_base_footprint(
  const PlannerParam & p, const bool ignore_offset = false)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  const auto front_offset = ignore_offset ? 0.0 : p.extra_front_offset;
  const auto rear_offset = ignore_offset ? 0.0 : p.extra_rear_offset;
  const auto right_offset = ignore_offset ? 0.0 : p.extra_right_offset;
  const auto left_offset = ignore_offset ? 0.0 : p.extra_left_offset;
  base_footprint.outer() = {
    {p.front_offset + front_offset, p.left_offset + left_offset},
    {p.front_offset + front_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.left_offset + left_offset}};
  return base_footprint;
}

inline lanelet::BasicPolygon2d project_to_pose(
  const tier4_autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

/// @brief calculate the path footprints
/// @details the resulting polygon follows the format used by the lanelet library: clockwise order
/// and implicit closing edge
/// @param [in] path input path
/// @param [in] first_idx first path index to consider
/// @param [in] params parameters
/// @return polygon footprints for each path point starting from first_idx
inline std::vector<lanelet::BasicPolygon2d> calculate_path_footprints(
  const EgoData & ego_data, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
  std::vector<lanelet::BasicPolygon2d> path_footprints;
  path_footprints.reserve(ego_data.path->points.size());
  for (auto i = ego_data.first_path_idx; i < ego_data.path->points.size(); ++i) {
    const auto & path_pose = ego_data.path->points[i].point.pose;
    const auto angle = tf2::getYaw(path_pose.orientation);
    const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
    lanelet::BasicPolygon2d footprint;
    for (const auto & p : rotated_footprint.outer())
      footprint.emplace_back(p.x() + path_pose.position.x, p.y() + path_pose.position.y);
    path_footprints.push_back(footprint);
  }
  return path_footprints;
}

inline lanelet::BasicPolygon2d calculate_current_ego_footprint(
  const EgoData & ego_data, const PlannerParam & params, const bool ignore_offset = false)
{
  const auto base_footprint = make_base_footprint(params, ignore_offset);
  const auto angle = tf2::getYaw(ego_data.pose.orientation);
  const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + ego_data.pose.position.x, p.y() + ego_data.pose.position.y);
  return footprint;
}

/// @brief calculate points along the path where we want ego to slowdown/stop
/// @param ego_data ego data (path, velocity, etc)
/// @param decisions decisions (before which point to stop, what lane to avoid entering, etc)
/// @param params parameters
/// @return precise points to insert in the path
inline std::vector<SlowdownToInsert> calculate_slowdown_points(
  const EgoData & ego_data, std::vector<Slowdown> & decisions, PlannerParam params)
{
  const auto can_decel = [&](const auto dist_ahead_of_ego, const auto target_vel) {
    const auto dist_to_target_vel =
      (ego_data.velocity * ego_data.velocity - target_vel * target_vel) / (2 * ego_data.max_decel);
    return dist_to_target_vel < dist_ahead_of_ego;
  };
  std::vector<SlowdownToInsert> to_insert;
  params.extra_front_offset += params.dist_buffer;
  const auto base_footprint = make_base_footprint(params);
  for (const auto & decision : decisions) {
    const auto & path_point = ego_data.path->points[decision.target_path_idx];
    if (decision.target_path_idx == 0) {
      const auto dist_ahead_of_ego = motion_utils::calcSignedArcLength(
        ego_data.path->points, ego_data.pose.position, path_point.point.pose.position);
      if (!params.skip_if_over_max_decel || can_decel(dist_ahead_of_ego, decision.velocity))
        to_insert.push_back({decision, path_point});
    } else {
      const auto & path_pose = path_point.point.pose;
      const auto & prev_path_pose = ego_data.path->points[decision.target_path_idx - 1].point.pose;

      const auto precision = 0.1;  // TODO(Maxime): param or better way to find no overlap pose
      auto interpolated_point = path_point;
      bool is_found = false;
      for (auto ratio = precision; ratio <= 1.0; ratio += precision) {
        interpolated_point.point.pose =
          tier4_autoware_utils::calcInterpolatedPose(path_pose, prev_path_pose, ratio, false);
        const auto is_overlap = boost::geometry::overlaps(
          project_to_pose(base_footprint, interpolated_point.point.pose),
          decision.lane_to_avoid.polygon2d().basicPolygon());
        if (!is_overlap) {
          const auto dist_ahead_of_ego = motion_utils::calcSignedArcLength(
            ego_data.path->points, ego_data.pose.position, path_point.point.pose.position);
          if (!params.skip_if_over_max_decel || can_decel(dist_ahead_of_ego, decision.velocity)) {
            to_insert.push_back({decision, path_point});
            is_found = true;
          }
          break;
        }
      }
      // if valid point not found, fallback to use the previous index (known to not overlap)
      if (!is_found)
        to_insert.push_back({decision, ego_data.path->points[decision.target_path_idx]});
    }
  }
  return to_insert;
}
}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_
