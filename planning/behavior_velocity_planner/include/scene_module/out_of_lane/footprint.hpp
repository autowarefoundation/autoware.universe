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
inline tier4_autoware_utils::Polygon2d make_base_footprint(const PlannerParam & p)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  base_footprint.outer() = {
    {p.front_offset + p.extra_front_offset, p.left_offset + p.extra_left_offset},
    {p.front_offset + p.extra_front_offset, p.right_offset - p.extra_right_offset},
    {p.rear_offset - p.extra_rear_offset, p.right_offset - p.extra_right_offset},
    {p.rear_offset - p.extra_rear_offset, p.left_offset + p.extra_left_offset}};
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
  const EgoInfo & ego_info, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
  std::vector<lanelet::BasicPolygon2d> path_footprints;
  path_footprints.reserve(ego_info.path.points.size());
  for (auto i = ego_info.first_path_idx; i < ego_info.path.points.size(); ++i) {
    const auto & path_pose = ego_info.path.points[i].point.pose;
    const auto angle = tf2::getYaw(path_pose.orientation);
    const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
    lanelet::BasicPolygon2d footprint;
    for (const auto & p : rotated_footprint.outer())
      footprint.emplace_back(p.x() + path_pose.position.x, p.y() + path_pose.position.y);
    path_footprints.push_back(footprint);
  }
  return path_footprints;
}

inline void insert_slowdown_points(
  autoware_auto_planning_msgs::msg::PathWithLaneId & path, const std::vector<Slowdown> & decisions,
  const PlannerParam & params)
{
  // TODO(Maxime): how do we handle more than max_decel ?
  const auto base_footprint = make_base_footprint(params);
  for (const auto & decision : decisions) {
    const auto & path_point = path.points[decision.target_path_idx];
    auto path_idx = decision.target_path_idx;
    if (decision.target_path_idx == 0) {
      planning_utils::insertVelocity(path, path_point, decision.velocity, path_idx);
      if (decision.velocity == 0.0) return;  // stop once a stop point is inserted
    } else {
      const auto & path_pose = path_point.point.pose;
      const auto & prev_path_pose = path.points[decision.target_path_idx - 1].point.pose;

      const auto precision = 0.1;  // TODO(Maxime): param or better way to find no overlap pose
      auto interpolated_point = path_point;
      for (auto ratio = precision; ratio <= 1.0; ratio += precision) {
        interpolated_point.point.pose =
          tier4_autoware_utils::calcInterpolatedPose(path_pose, prev_path_pose, ratio, false);
        const auto overlaps = boost::geometry::overlaps(
          project_to_pose(base_footprint, interpolated_point.point.pose),
          decision.lane_to_avoid.polygon2d().basicPolygon());
        if (!overlaps) {
          planning_utils::insertVelocity(path, interpolated_point, decision.velocity, path_idx);
          if (decision.velocity == 0.0) return;  // stop once a stop point is inserted
        }
      }
    }
  }
}
}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__FOOTPRINT_HPP_
