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

#ifndef SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
#define SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_

#include "scene_module/out_of_lane/overlapping_range.hpp"
#include "scene_module/out_of_lane/types.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace out_of_lane_utils
{
inline double distance_along_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & ego_path, const size_t first_idx,
  const size_t target_idx)
{
  auto s = 0.0;
  for (auto i = first_idx; i <= target_idx && i + 1 < ego_path.points.size(); ++i)
    s += tier4_autoware_utils::calcDistance2d(ego_path.points[i], ego_path.points[i + 1]);
  return s;
}

inline std::pair<double, double> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<route_handler::RouteHandler> route_handler)
{
  const auto & p = object.kinematics.initial_pose_with_covariance.pose.position;
  const auto object_point = lanelet::BasicPoint2d(p.x, p.y);
  const auto half_size = object.shape.dimensions.x / 2;
  lanelet::ConstLanelets object_lanelets;
  for (const auto & ll : lanelets)
    if (boost::geometry::within(object_point, ll.polygon2d().basicPolygon()))
      object_lanelets.push_back(ll);
  auto min_dist = std::numeric_limits<double>::infinity();
  auto max_dist = 0.0;
  for (const auto & lane : object_lanelets) {
    const auto path = route_handler->getRoutingGraphPtr()->shortestPath(lane, range.lane);
    std::printf(
      "\t\t\tPath ? %d [from %ld to %ld]\n", path.has_value(), lane.id(), range.lane.id());
    if (path) {
      lanelet::ConstLanelets lls;
      for (const auto & ll : *path) lls.push_back(ll);
      geometry_msgs::msg::Pose p;
      p.position.x = object_point.x();
      p.position.y = object_point.y();
      const auto object_length = lanelet::utils::getArcCoordinates(lls, p).length;
      p.position.x = range.entering_point.x();
      p.position.y = range.entering_point.y();
      const auto enter_length = lanelet::utils::getArcCoordinates(lls, p).length;
      p.position.x = range.exiting_point.x();
      p.position.y = range.exiting_point.y();
      const auto exit_length = lanelet::utils::getArcCoordinates(lls, p).length;
      std::printf(
        "\t\t\t%2.2f -> [%2.2f(%2.2f, %2.2f) - %2.2f(%2.2f, %2.2f)]\n", object_length, enter_length,
        range.entering_point.x(), range.entering_point.y(), exit_length, range.exiting_point.x(),
        range.exiting_point.y());
      const auto & [object_enter_length, object_exit_length] =
        std::minmax(enter_length, exit_length);
      min_dist = std::min(min_dist, object_enter_length - object_length);
      max_dist = std::max(max_dist, object_exit_length - object_length);
    }
  }
  const auto v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  return std::make_pair((min_dist - half_size) / v, (max_dist + half_size) / v);
}

inline std::vector<Slowdown> calculate_decisions(
  const OverlapRanges & ranges, const autoware_auto_planning_msgs::msg::PathWithLaneId & ego_path,
  const size_t first_idx, const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const std::shared_ptr<route_handler::RouteHandler> route_handler,
  const lanelet::ConstLanelets & lanelets, const PlannerParam & params, DebugData & debug)
{
  // TODO(Maxime): move to fn, improve time estimatate.
  const auto time_along_path = [&](const auto & idx) {
    auto t = 0.0;
    for (auto i = first_idx; i <= idx && i + 1 < ego_path.points.size(); ++i) {
      const auto ds =
        tier4_autoware_utils::calcDistance2d(ego_path.points[i], ego_path.points[i + 1]);
      const auto v = ego_path.points[i].point.longitudinal_velocity_mps * 0.2;  // TODO(Maxime) TMP
      t += ds / v;
    }
    return t;
  };
  std::vector<Slowdown> decisions;
  std::cout << "** Decisions\n";
  for (const auto & range : ranges) {
    // skip if we already entered the range
    if (range.entering_path_idx == 0UL) continue;

    const auto ego_enter_time = time_along_path(range.entering_path_idx);
    const auto ego_exit_time = time_along_path(range.exiting_path_idx);
    const auto ego_dist_to_range =
      distance_along_path(ego_path, first_idx, range.entering_path_idx);
    // TODO(Maxime): remove debug ?
    debug.ranges.push_back(range);
    debug.ego_times.emplace_back(ego_enter_time, ego_exit_time);
    auto & npc_times = debug.npc_times.emplace_back();

    std::printf(
      "\t[%lu -> %lu] %ld (ego enters at %2.2f, exits at %2.2f)\n", range.entering_path_idx,
      range.exiting_path_idx, range.lane.id(), ego_enter_time, ego_exit_time);
    auto min_object_enter_time = std::numeric_limits<double>::max();
    auto max_object_exit_time = 0.0;
    for (const auto & object : objects.objects) {
      auto & debug_pair = npc_times.emplace_back(0.0, 0.0);
      if (object.kinematics.initial_twist_with_covariance.twist.linear.x < params.objects_min_vel)
        continue;  // skip objects with velocity bellow a threshold
      const auto & [enter_time, exit_time] =
        object_time_to_range(object, range, lanelets, route_handler);
      debug_pair.first = enter_time;
      debug_pair.second = exit_time;
      std::printf(
        "\t\t[%s] going at %2.2fm/s enter at %2.2fs, exits at %2.2fs\n",
        tier4_autoware_utils::toHexString(object.object_id).c_str(),
        object.kinematics.initial_twist_with_covariance.twist.linear.x, enter_time, exit_time);
      min_object_enter_time = std::min(min_object_enter_time, std::max(0.0, enter_time));
      max_object_exit_time = std::max(max_object_exit_time, exit_time);
    }
    const auto incoming_objects = min_object_enter_time < max_object_exit_time;
    const auto threshold_condition =
      params.mode == "threshold" && min_object_enter_time < params.time_threshold;
    const auto ego_enters_before_object = ego_enter_time + params.intervals_ego_buffer <
                                          min_object_enter_time + params.intervals_obj_buffer;
    const auto ego_exits_after_object = ego_exit_time + params.intervals_ego_buffer >
                                        max_object_exit_time + params.intervals_obj_buffer;
    const auto interval_condition = ego_enters_before_object && ego_exits_after_object;
    constexpr auto ttc = 2.0;  // TODO(Maxime): calculate ttc
    const auto ttc_condition = ttc <= params.ttc_threshold;
    const auto should_take_action =
      incoming_objects && (threshold_condition || interval_condition || ttc_condition);
    Slowdown decision;
    decision.target_path_idx = first_idx + range.entering_path_idx;  // add offset from curr pose
    decision.lane_to_avoid = range.lane;
    if (should_take_action) {
      if (ego_dist_to_range < params.stop_dist_threshold) {
        std::printf("\t\tWill stop\n");
        decision.velocity = 0.0;
        decisions.push_back(decision);
      } else if (ego_dist_to_range < params.slow_dist_threshold) {
        std::printf("\t\tWill slow\n");
        decision.velocity = params.slow_velocity;
        decisions.push_back(decision);
      }
    }
  }
  return decisions;
}

}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
