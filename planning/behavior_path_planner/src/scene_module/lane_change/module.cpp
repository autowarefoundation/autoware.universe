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

#include "behavior_path_planner/scene_module/lane_change/interface.hpp"
#include "behavior_path_planner/scene_module/lane_change/module_template.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/util/lane_change/util.hpp"
#include "behavior_path_planner/util/path_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
LaneChangeModuleTemplate::LaneChangeModuleTemplate(
  const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction)
: parameters_{parameters}, direction_{direction}
{
}

void LaneChangeModuleTemplate::updateLaneChangeStatus(
  const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & previous_module_path)
{
  status_.current_lanes = util::getCurrentLanesFromPath(prev_module_reference_path, planner_data_);
  status_.lane_change_lanes = util::lane_change::getLaneChangeLanes(
    planner_data_, status_.current_lanes, lane_change_lane_length_, parameters_->prepare_duration,
    direction_, type_);

  // Find lane change path
  const auto [found_valid_path, found_safe_path] =
    getSafePath(prev_module_reference_path, previous_module_path, status_.lane_change_path);

  // Update status
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = util::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = util::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

#ifdef USE_OLD_ARCHITECTURE
lanelet::ConstLanelets LaneChangeModuleTemplate::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto minimum_lane_change_length = planner_data_->parameters.minimum_lane_changing_length;
  const auto lane_change_prepare_duration = parameters_->prepare_duration;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * lane_change_prepare_duration, minimum_lane_change_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  const auto lane_change_lane = route_handler->getLaneChangeTarget(current_check_lanes);
  if (lane_change_lane) {
    lane_change_lanes = route_handler->getLaneletSequence(
      lane_change_lane.get(), current_pose, lane_change_lane_length, lane_change_lane_length);
  } else {
    lane_change_lanes.clear();
  }

  return lane_change_lanes;
}
#endif

std::pair<bool, bool> LaneChangeModuleTemplate::getSafePath(
  const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & prev_module_path,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto & common_parameters = planner_data_->parameters;

  const auto current_lanes =
    util::getCurrentLanesFromPath(prev_module_reference_path, planner_data_);

  const auto lane_change_lanes = util::lane_change::getLaneChangeLanes(
    planner_data_, current_lanes, lane_change_lane_length_, parameters_->prepare_duration,
    direction_, type_);

  if (lane_change_lanes.empty()) {
    return std::make_pair(false, false);
  }

  // find candidate paths
  LaneChangePaths valid_paths;

  const auto [found_valid_path, found_safe_path] = util::lane_change::getLaneChangePaths(
    prev_module_path, *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
    planner_data_->dynamic_object, common_parameters, *parameters_, check_distance_, direction_,
    &valid_paths, &object_debug_);

  if (!found_valid_path) {
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  return {found_valid_path, found_safe_path};
}

bool LaneChangeModuleTemplate::isSafe() const
{
  return status_.is_safe;
}

bool LaneChangeModuleTemplate::isValidPath() const
{
  return status_.is_valid_path;
}

bool LaneChangeModuleTemplate::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;

  // check lane departure
  const auto drivable_lanes = util::lane_change::generateDrivableLanes(
    *route_handler, util::extendLanes(route_handler, status_.current_lanes),
    util::extendLanes(route_handler, status_.lane_change_lanes));
  const auto expanded_lanes = util::expandLanelets(
    drivable_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  const auto lanelets = util::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_in_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lanelet)) {
        is_in_lanelet = true;
        break;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }

  // check relative angle
  if (!util::checkPathRelativeAngle(path, M_PI)) {
    return false;
  }

  return true;
}

bool LaneChangeModuleTemplate::isNearEndOfLane() const
{
  const auto & current_pose = getEgoPose();
  const double threshold = util::calcTotalLaneChangeLength(planner_data_->parameters);

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

PathWithLaneId LaneChangeModuleTemplate::generatePlannedPath(
  const std::vector<DrivableLanes> & prev_drivable_lanes)
{
  auto path = getLaneChangePath().path;
  generateExtendedDrivableArea(prev_drivable_lanes, path);

  if (isAbortState()) {
    return path;
  }

  if (isStopState()) {
    const auto stop_point = util::insertStopPoint(0.1, path);
  }

  return path;
}

bool LaneChangeModuleTemplate::isCurrentSpeedLow() const
{
  constexpr double threshold_ms = 10.0 * 1000 / 3600;
  return getEgoTwist().linear.x < threshold_ms;
}

bool LaneChangeModuleTemplate::isCancelConditionSatisfied()
{
  current_lane_change_state_ = LaneChangeStates::Normal;

  if (!parameters_->enable_cancel_lane_change) {
    return false;
  }

  Pose ego_pose_before_collision;
  const auto is_path_safe = isApprovedPathSafe(ego_pose_before_collision);

  if (!is_path_safe) {
    const auto & common_parameters = planner_data_->parameters;
    const bool is_within_original_lane = util::lane_change::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), common_parameters);

    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_lane_change_state_ = LaneChangeStates::Stop;
      return false;
    }

    if (is_within_original_lane) {
      current_lane_change_state_ = LaneChangeStates::Cancel;
      return true;
    }

    if (!parameters_->enable_abort_lane_change) {
      return false;
    }

    return isAbortConditionSatisfied(ego_pose_before_collision);
  }

  return false;
}

bool LaneChangeModuleTemplate::isAbortConditionSatisfied(const Pose & pose)
{
  const auto & common_parameters = planner_data_->parameters;

  const auto found_abort_path = util::lane_change::getAbortPaths(
    planner_data_, status_.lane_change_path, pose, common_parameters, *parameters_);

  if (!found_abort_path && !is_abort_path_approved_) {
    current_lane_change_state_ = LaneChangeStates::Stop;
    return true;
  }

  current_lane_change_state_ = LaneChangeStates::Abort;

  if (!is_abort_path_approved_) {
    abort_path_ = std::make_shared<LaneChangePath>(*found_abort_path);
  }

  return true;
}

PathWithLaneId LaneChangeModuleTemplate::getReferencePath() const
{
  return util::getCenterLinePathFromRootLanelet(status_.lane_change_lanes.front(), planner_data_);
}

bool LaneChangeModuleTemplate::isStopState() const
{
  return current_lane_change_state_ == LaneChangeStates::Stop;
}

bool LaneChangeModuleTemplate::isAbortState() const
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

bool LaneChangeModuleTemplate::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_path = status_.lane_change_path.path;
  const auto & lane_change_end = status_.lane_change_path.shift_line.end;
  const double dist_to_lane_change_end = motion_utils::calcSignedArcLength(
    lane_change_path.points, current_pose.position, lane_change_end.position);
  return dist_to_lane_change_end + parameters_->lane_change_finish_judge_buffer < 0.0;
}

Pose LaneChangeModuleTemplate::getEgoPose() const
{
  return planner_data_->self_odometry->pose.pose;
}
Twist LaneChangeModuleTemplate::getEgoTwist() const
{
  return planner_data_->self_odometry->twist.twist;
}
std_msgs::msg::Header LaneChangeModuleTemplate::getRouteHeader() const
{
  return planner_data_->route_handler->getRouteHeader();
}
void LaneChangeModuleTemplate::generateExtendedDrivableArea(
  const std::vector<DrivableLanes> & prev_drivable_lanes, PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  const auto drivable_lanes = util::lane_change::generateDrivableLanes(
    prev_drivable_lanes, *route_handler, status_.current_lanes, status_.lane_change_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);
  util::generateDrivableArea(path, expanded_lanes, common_parameters.vehicle_length, planner_data_);
}

bool LaneChangeModuleTemplate::isApprovedPathSafe(Pose & ego_pose_before_collision) const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;
  const auto & lane_change_parameters = *parameters_;
  const auto & route_handler = planner_data_->route_handler;
  const auto & path = status_.lane_change_path;

  // get lanes used for detection
  const auto check_lanes = util::lane_change::getExtendedTargetLanesForCollisionCheck(
    *route_handler, path.target_lanelets.front(), current_pose, check_distance_);

  std::unordered_map<std::string, CollisionCheckDebug> debug_data;
  const auto lateral_buffer =
    util::lane_change::calcLateralBufferForFiltering(common_parameters.vehicle_width);
  const auto dynamic_object_indices = util::lane_change::filterObjectIndices(
    {path}, *dynamic_objects, check_lanes, current_pose, common_parameters.forward_path_length,
    lane_change_parameters, lateral_buffer);

  return util::lane_change::isLaneChangePathSafe(
    path, dynamic_objects, dynamic_object_indices, current_pose, current_twist, common_parameters,
    *parameters_, common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, ego_pose_before_collision, debug_data,
    status_.lane_change_path.acceleration);
}

TurnSignalInfo LaneChangeModuleTemplate::updateOutputTurnSignal()
{
  calcTurnSignalInfo();
  TurnSignalInfo turn_signal_info;
  const auto [turn_signal_command, distance_to_vehicle_front] = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  turn_signal_info.turn_signal.command = turn_signal_command.command;

  turn_signal_info.desired_start_point =
    status_.lane_change_path.turn_signal_info.desired_start_point;
  turn_signal_info.required_start_point =
    status_.lane_change_path.turn_signal_info.required_start_point;
  turn_signal_info.required_end_point =
    status_.lane_change_path.turn_signal_info.required_end_point;
  turn_signal_info.desired_end_point = status_.lane_change_path.turn_signal_info.desired_end_point;

  return turn_signal_info;
}

void LaneChangeModuleTemplate::calcTurnSignalInfo()
{
  const auto get_blinker_pose =
    [](const PathWithLaneId & path, const lanelet::ConstLanelets & lanes, const double length) {
      const auto & points = path.points;
      const auto arc_front = lanelet::utils::getArcCoordinates(lanes, points.front().point.pose);
      for (const auto & point : points) {
        const auto & pt = point.point.pose;
        const auto arc_current = lanelet::utils::getArcCoordinates(lanes, pt);
        const auto diff = arc_current.length - arc_front.length;
        if (diff > length) {
          return pt;
        }
      }

      return points.front().point.pose;
    };

  const auto & path = status_.lane_change_path;
  TurnSignalInfo turn_signal_info{};

  turn_signal_info.desired_start_point = std::invoke([&]() {
    const auto blinker_start_duration = planner_data_->parameters.turn_signal_search_time;
    const auto prepare_duration = parameters_->prepare_duration;
    const auto prepare_to_blinker_start_diff = prepare_duration - blinker_start_duration;
    if (prepare_to_blinker_start_diff < 1e-5) {
      return path.path.points.front().point.pose;
    }

    return get_blinker_pose(path.path, path.reference_lanelets, prepare_to_blinker_start_diff);
  });
  turn_signal_info.desired_end_point = path.shift_line.end;

  turn_signal_info.required_start_point = path.shift_line.start;
  const auto mid_lane_change_length = path.length.prepare / 2;
  const auto & shifted_path = path.shifted_path.path;
  turn_signal_info.required_end_point =
    get_blinker_pose(shifted_path, path.target_lanelets, mid_lane_change_length);

  status_.lane_change_path.turn_signal_info = turn_signal_info;
}

void LaneChangeModuleTemplate::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  object_debug_.clear();
}
}  // namespace behavior_path_planner
