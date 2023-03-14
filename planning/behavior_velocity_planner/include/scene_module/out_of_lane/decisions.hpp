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

#include "scene_module/out_of_lane/out_of_lane_utils.hpp"
#include "scene_module/out_of_lane/overlapping_range.hpp"
#include "scene_module/out_of_lane/types.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane_utils
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

/// @brief estimate the times when an object will enter and exit a given overlapping range
/// @details the enter/exit is relative to ego and may be inversed if the object drives in the
/// opposite direction
inline std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler)
{
  const auto & p = object.kinematics.initial_pose_with_covariance.pose.position;
  const auto object_point = lanelet::BasicPoint2d(p.x, p.y);
  const auto half_size = object.shape.dimensions.x / 2;
  lanelet::ConstLanelets object_lanelets;
  for (const auto & ll : lanelets)
    if (boost::geometry::within(object_point, ll.polygon2d().basicPolygon()))
      object_lanelets.push_back(ll);
  auto enter_arc_length = std::optional<double>();
  auto exit_arc_length = std::optional<double>();
  for (const auto & lane : object_lanelets) {
    const auto path = route_handler->getRoutingGraphPtr()->shortestPath(lane, range.lane);
    std::printf(
      "\t\t\tPath ? %d [from %ld to %ld]\n", path.has_value(), lane.id(), range.lane.id());
    if (path) {
      lanelet::ConstLanelets lls;
      for (const auto & ll : *path) lls.push_back(ll);
      geometry_msgs::msg::Pose p;
      p.position.set__x(object_point.x()).set__y(object_point.y());
      const auto object_curr_length = lanelet::utils::getArcCoordinates(lls, p).length;
      p.position.set__x(range.entering_point.x()).set__y(range.entering_point.y());
      const auto enter_length = lanelet::utils::getArcCoordinates(lls, p).length;
      p.position.set__x(range.exiting_point.x()).set__y(range.exiting_point.y());
      const auto exit_length = lanelet::utils::getArcCoordinates(lls, p).length;
      std::printf(
        "\t\t\t%2.2f -> [%2.2f(%2.2f, %2.2f) - %2.2f(%2.2f, %2.2f)]\n", object_curr_length,
        enter_length, range.entering_point.x(), range.entering_point.y(), exit_length,
        range.exiting_point.x(), range.exiting_point.y());
      // multiple paths to the overlap -> be conservative and use the "worst" case
      // "worst" = min/max arc length depending on if the lane is running opposite to the ego path
      const auto is_opposite = enter_length > exit_length;
      if (!enter_arc_length)
        enter_arc_length = enter_length;
      else if (is_opposite)
        enter_arc_length = std::max(*enter_arc_length, enter_length);
      else
        enter_arc_length = std::min(*enter_arc_length, enter_length);
      if (!exit_arc_length)
        exit_arc_length = exit_length;
      else if (is_opposite)
        exit_arc_length = std::max(*exit_arc_length, exit_length);
      else
        exit_arc_length = std::min(*exit_arc_length, exit_length);
    }
  }
  if (enter_arc_length && exit_arc_length) {
    const auto v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    return std::make_pair((*enter_arc_length - half_size) / v, (*exit_arc_length + half_size) / v);
  }
  return {};
}

inline double time_along_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & ego_path, const size_t from_idx,
  const size_t to_idx)
{
  auto t = 0.0;
  for (auto i = from_idx; i <= to_idx && i + 1 < ego_path.points.size(); ++i) {
    const auto ds =
      tier4_autoware_utils::calcDistance2d(ego_path.points[i], ego_path.points[i + 1]);
    const auto v = ego_path.points[i].point.longitudinal_velocity_mps * 0.2;  // TODO(Maxime) TMP
    t += ds / v;
  }
  return t;
}

inline std::vector<Slowdown> calculate_decisions(
  const OverlapRanges & ranges, const autoware_auto_planning_msgs::msg::PathWithLaneId & ego_path,
  const size_t first_idx, const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const std::shared_ptr<route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanelets, const PlannerParam & params, DebugData & debug)
{
  std::vector<Slowdown> decisions;
  std::cout << "** Decisions\n";
  for (const auto & range : ranges) {
    if (range.entering_path_idx == 0UL) continue;  // skip if we already entered the range
    bool should_not_enter = false;  // we will decide if we need to stop/slow before this range

    const auto ego_enter_time = time_along_path(ego_path, first_idx, range.entering_path_idx);
    const auto ego_exit_time = time_along_path(ego_path, first_idx, range.exiting_path_idx);
    const auto ego_dist_to_range =
      distance_along_path(ego_path, first_idx, range.entering_path_idx);
    std::printf(
      "\t[%lu -> %lu] %ld | ego dist = %2.2f (enters at %2.2f, exits at %2.2f)\n",
      range.entering_path_idx, range.exiting_path_idx, range.lane.id(), ego_dist_to_range,
      ego_enter_time, ego_exit_time);

    // TODO(Maxime): remove debug ?
    debug.ranges.push_back(range);
    debug.ego_times.emplace_back(ego_enter_time, ego_exit_time);
    auto & npc_times = debug.npc_times.emplace_back();

    for (const auto & object : objects.objects) {
      auto & debug_pair = npc_times.emplace_back(0.0, 0.0);
      if (object.kinematics.initial_twist_with_covariance.twist.linear.x < params.objects_min_vel)
        continue;  // skip objects with velocity bellow a threshold
      const auto enter_exit_time = object_time_to_range(object, range, lanelets, route_handler);
      if (!enter_exit_time) continue;  // object is not driving towards the overlapping range

      const auto & [enter_time, exit_time] = *enter_exit_time;
      debug_pair.first = std::min(enter_time, exit_time);
      debug_pair.second = std::max(enter_time, exit_time);
      std::printf(
        "\t\t[%s] going at %2.2fm/s enter at %2.2fs, exits at %2.2fs\n",
        tier4_autoware_utils::toHexString(object.object_id).c_str(),
        object.kinematics.initial_twist_with_covariance.twist.linear.x, enter_time, exit_time);

      const auto object_is_going_opposite_way = enter_time > exit_time;
      if (params.mode == "threshold") {
        if (std::min(enter_time, exit_time) < params.time_threshold) should_not_enter = true;
      } else if (params.mode == "intervals") {
        if (object_is_going_opposite_way) {
          const auto ego_exits_before_object_enters =
            ego_exit_time + params.intervals_ego_buffer < enter_time + params.intervals_obj_buffer;
          if (ego_exits_before_object_enters) should_not_enter = true;
        } else {
          const auto ego_enters_before_object =
            ego_enter_time + params.intervals_ego_buffer < enter_time + params.intervals_obj_buffer;
          const auto ego_exits_after_object =
            ego_exit_time + params.intervals_ego_buffer > exit_time + params.intervals_obj_buffer;
          if (ego_enters_before_object && ego_exits_after_object) should_not_enter = true;
        }
      } else if (params.mode == "ttc") {
        auto ttc = 0.0;
        const auto ttc_at_enter =
          ego_enter_time - (object_is_going_opposite_way ? exit_time : enter_time);
        const auto ttc_at_exit =
          ego_exit_time - (object_is_going_opposite_way ? enter_time : exit_time);
        // one vehicle is predicted to overtake the other -> collision
        if (std::signbit(ttc_at_enter) != std::signbit(ttc_at_exit))
          ttc = 0.0;
        else
          ttc = std::min(ttc_at_enter, ttc_at_exit);
        if (ttc <= params.ttc_threshold) should_not_enter = true;
      }
    }
    if (should_not_enter) {
      Slowdown decision;
      decision.target_path_idx = first_idx + range.entering_path_idx;  // add offset from curr pose
      decision.lane_to_avoid = range.lane;
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

}  // namespace behavior_velocity_planner::out_of_lane_utils

#endif  // SCENE_MODULE__OUT_OF_LANE__DECISIONS_HPP_
