// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/pull_over_planner/bezier_pull_over.hpp"

namespace autoware::behavior_path_planner
{
BezierPullOver::BezierPullOver(
  rclcpp::Node & node, const GoalPlannerParameters & parameters,
  const LaneDepartureChecker & lane_departure_checker)
: PullOverPlannerBase{node, parameters},
  lane_departure_checker_{lane_departure_checker},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
}

std::optional<PullOverPath> BezierPullOver::plan(
  [[maybe_unused]] const GoalCandidate & modified_goal_pose, [[maybe_unused]] const size_t id,
  [[maybe_unused]] const std::shared_ptr<const PlannerData> planner_data,
  [[maybe_unused]] const BehaviorModuleOutput & previous_module_output)
{
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
