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

#include "autoware/behavior_velocity_blind_spot_module/scene.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

namespace autoware::behavior_velocity_planner
{

/*
 * for default
 */
template <typename T>
void BlindSpotModule::setRTCStatusByDecision(
  const T &, [[maybe_unused]] const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  static_assert("Unsupported type passed to setRTCStatus");
  return;
}

template <typename T>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const T & decision,
  [[maybe_unused]] tier4_planning_msgs::msg::PathWithLaneId * path)
{
  static_assert("Unsupported type passed to reactRTCApprovalByDecision");
}

/*
 * for InternalError
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  [[maybe_unused]] const InternalError & decision,
  [[maybe_unused]] const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const InternalError & decision,
  [[maybe_unused]] tier4_planning_msgs::msg::PathWithLaneId * path)
{
  return;
}

/*
 * For OverPassJudge
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  [[maybe_unused]] const OverPassJudge & decision,
  [[maybe_unused]] const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  [[maybe_unused]] const OverPassJudge & decision,
  [[maybe_unused]] tier4_planning_msgs::msg::PathWithLaneId * path)
{
  return;
}

/*
 * for Unsafe
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  const Unsafe & decision, const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  setSafe(false);
  const auto & current_pose = planner_data_->current_odometry->pose;
  setDistance(autoware::motion_utils::calcSignedArcLength(
    path.points, current_pose.position, decision.stop_line_idx));
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  const Unsafe & decision, tier4_planning_msgs::msg::PathWithLaneId * path)
{
  if (!isActivated()) {
    constexpr double stop_vel = 0.0;
    planning_utils::setVelocityFromIndex(decision.stop_line_idx, stop_vel, path);
    debug_data_.virtual_wall_pose = planning_utils::getAheadPose(
      decision.stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);

    const auto stop_pose = path->points.at(decision.stop_line_idx).point.pose;
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/,
      "blind_spot(module is judging as UNSAFE)");
  }
  return;
}

/*
 * for Safe
 */
template <>
void BlindSpotModule::setRTCStatusByDecision(
  const Safe & decision, const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  setSafe(true);
  const auto & current_pose = planner_data_->current_odometry->pose;
  setDistance(autoware::motion_utils::calcSignedArcLength(
    path.points, current_pose.position, decision.stop_line_idx));
  return;
}

template <>
void BlindSpotModule::reactRTCApprovalByDecision(
  const Safe & decision, tier4_planning_msgs::msg::PathWithLaneId * path)
{
  if (!isActivated()) {
    constexpr double stop_vel = 0.0;
    planning_utils::setVelocityFromIndex(decision.stop_line_idx, stop_vel, path);
    debug_data_.virtual_wall_pose = planning_utils::getAheadPose(
      decision.stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);

    const auto stop_pose = path->points.at(decision.stop_line_idx).point.pose;
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/,
      "blind_spot(module is judging as SAFE and RTC is not approved)");
  }
  return;
}

}  // namespace autoware::behavior_velocity_planner
