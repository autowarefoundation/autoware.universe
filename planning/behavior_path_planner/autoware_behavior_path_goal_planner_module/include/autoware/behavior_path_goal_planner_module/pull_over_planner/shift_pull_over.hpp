// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER__SHIFT_PULL_OVER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER__SHIFT_PULL_OVER_HPP_

#include "autoware/behavior_path_goal_planner_module/pull_over_planner/pull_over_planner_base.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::lane_departure_checker::LaneDepartureChecker;

class ShiftPullOver : public PullOverPlannerBase
{
public:
  ShiftPullOver(rclcpp::Node & node, const GoalPlannerParameters & parameters);
  PullOverPlannerType getPlannerType() const override { return PullOverPlannerType::SHIFT; };
  std::optional<PullOverPath> plan(
    const GoalCandidate & modified_goal_pose, const size_t id,
    const std::shared_ptr<const PlannerData> planner_data,
    const BehaviorModuleOutput & upstream_module_output) override;

protected:
  PathWithLaneId generateReferencePath(
    const std::shared_ptr<const PlannerData> planner_data,
    const lanelet::ConstLanelets & road_lanes, const Pose & end_pose) const;
  std::optional<PathWithLaneId> cropPrevModulePath(
    const PathWithLaneId & prev_module_path, const Pose & shift_end_pose) const;
  std::optional<PullOverPath> generatePullOverPath(
    const GoalCandidate & goal_candidate, const size_t id,
    const std::shared_ptr<const PlannerData> planner_data,
    const BehaviorModuleOutput & upstream_module_output, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & pull_over_lanes, const double lateral_jerk) const;
  static double calcBeforeShiftedArcLength(
    const PathWithLaneId & path, const double after_shifted_arc_length, const double dr);
  static std::vector<double> splineTwoPoints(
    std::vector<double> base_s, std::vector<double> base_x, const double begin_diff,
    const double end_diff, std::vector<double> new_s);
  static std::vector<Pose> interpolatePose(
    const Pose & start_pose, const Pose & end_pose, const double resample_interval);

  const LaneDepartureChecker lane_departure_checker_;

  const bool left_side_parking_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER__SHIFT_PULL_OVER_HPP_
