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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/util/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/util/lane_change/util.hpp"
#include "behavior_path_planner/util/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebugMap;
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class LaneChangeBase
{
public:
  LaneChangeBase(const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction)
  : parameters_{parameters}, direction_{direction}
  {
  }

  virtual void updateLaneChangeStatus(
    const PathWithLaneId & prev_module_reference_path,
    const PathWithLaneId & previous_module_path) = 0;

  virtual std::pair<bool, bool> getSafePath(
    const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & prev_module_path,
    LaneChangePath & safe_path) const = 0;

  virtual PathWithLaneId generatePlannedPath(
    const std::vector<DrivableLanes> & prev_drivable_lanes) = 0;

  virtual void generateExtendedDrivableArea(
    const std::vector<DrivableLanes> & prev_drivable_lanes, PathWithLaneId & path) = 0;

  virtual bool hasFinishedLaneChange() const = 0;

  virtual PathWithLaneId getReferencePath() const = 0;

  virtual bool isCancelConditionSatisfied() = 0;

  virtual bool isAbortConditionSatisfied(const Pose & pose) = 0;

  virtual void resetParameters() = 0;

  virtual TurnSignalInfo updateOutputTurnSignal() = 0;

  const LaneChangeStatus & getLaneChangeStatus() const { return status_; }

  LaneChangePath getLaneChangePath() const
  {
    return isAbortState() ? *abort_path_ : status_.lane_change_path;
  }

  const LaneChangePaths & getDebugValidPath() const { return debug_valid_path_; }

  const CollisionCheckDebugMap & getDebugData() const { return object_debug_; }

  bool isAbortState() const
  {
    if (!parameters_->enable_abort_lane_change) {
      return false;
    }

    const auto is_within_current_lane = util::lane_change::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), planner_data_->parameters);

    if (!is_within_current_lane) {
      return false;
    }

    if (current_lane_change_state_ != LaneChangeStates::Abort) {
      return false;
    }

    if (!abort_path_) {
      return false;
    }

    return true;
  }

  bool isSafe() const { return status_.is_safe; }

  bool isStopState() const { return current_lane_change_state_ == LaneChangeStates::Stop; }

  bool isValidPath() const { return status_.is_valid_path; }

  std_msgs::msg::Header getRouteHeader() const
  {
    return planner_data_->route_handler->getRouteHeader();
  }

  void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }

  const Pose & getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  const Point & getEgoPosition() const { return getEgoPose().position; }

  const Twist & getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }

  double getEgoVelocity() const { return getEgoTwist().linear.x; }

  const Direction & getDirection() const { return direction_; }

protected:
  virtual bool isApprovedPathSafe(Pose & ego_pose_before_collision) const = 0;

  virtual void calcTurnSignalInfo() = 0;

  virtual bool isValidPath(const PathWithLaneId & path) const = 0;

  virtual bool isNearEndOfLane() const = 0;

  virtual bool isCurrentSpeedLow() const = 0;

  LaneChangeStatus status_{};
  PathShifter path_shifter_{};

  LaneChangeStates current_lane_change_state_{};

  std::shared_ptr<LaneChangeParameters> parameters_{};
  std::shared_ptr<LaneChangePath> abort_path_{};
  std::shared_ptr<const PlannerData> planner_data_{};

  PathWithLaneId prev_approved_path_{};

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};

  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_activated_{false};

  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};

  mutable CollisionCheckDebugMap object_debug_{};
  mutable LaneChangePaths debug_valid_path_{};
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_
