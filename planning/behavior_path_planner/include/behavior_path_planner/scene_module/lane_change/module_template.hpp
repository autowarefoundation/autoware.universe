// Copyright 2023 TIER IV, Inc.
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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MODULE_TEMPLATE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MODULE_TEMPLATE_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/util/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class LaneChangeModuleTemplate
{
public:
  LaneChangeModuleTemplate(
    const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction);

  void updateLaneChangeStatus(
    const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & previous_module_path);

  std::pair<bool, bool> getSafePath(
    const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & prev_module_path,
    LaneChangePath & safe_path) const;

  const LaneChangeStatus & getLaneChangeStatus() const { return status_; }

  LaneChangePath getLaneChangePath()
  {
    if (isAbortState()) {
      return *abort_path_;
    }

    return status_.lane_change_path;
  }

  void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }

  PathWithLaneId generatePlannedPath(const std::vector<DrivableLanes> & prev_drivable_lanes);

  void generateExtendedDrivableArea(
    const std::vector<DrivableLanes> & prev_drivable_lanes, PathWithLaneId & path);

  bool isValidPath() const;
  bool hasFinishedLaneChange() const;
  bool isStopState() const;
  bool isAbortState() const;
  PathWithLaneId getReferencePath() const;
  void resetParameters();
  const Point & getEgoPosition() const { return planner_data_->self_odometry->pose.pose.position; }
  const std::unordered_map<std::string, CollisionCheckDebug> & getDebugData() const
  {
    return object_debug_;
  }
  const LaneChangePaths & getDebugValidPath() const { return debug_valid_path_; }
  bool isCancelConditionSatisfied();
  bool isAbortConditionSatisfied(const Pose & pose);
  Pose getEgoPose() const;
  const Direction & getDirection() const { return direction_; }
  TurnSignalInfo updateOutputTurnSignal();

private:
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;

  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);
  bool isApprovedPathSafe(Pose & ego_pose_before_collision) const;
  void calcTurnSignalInfo();

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  bool isSafe() const;
  bool isValidPath(const PathWithLaneId & path) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;

  // getter
  Twist getEgoTwist() const;
  std_msgs::msg::Header getRouteHeader() const;
  void resetPathIfAbort();

  std::shared_ptr<LaneChangeParameters> parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;
  LaneChangeStates current_lane_change_state_;
  std::shared_ptr<LaneChangePath> abort_path_;
  PathWithLaneId prev_approved_path_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_activated_{false};

  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};
  std::shared_ptr<const PlannerData> planner_data_;

  // debug
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable LaneChangePaths debug_valid_path_;
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MODULE_TEMPLATE_HPP_
