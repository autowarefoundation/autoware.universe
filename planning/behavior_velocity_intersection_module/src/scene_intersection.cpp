// Copyright 2020 Tier IV, Inc.
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

#include "scene_intersection.hpp"

#include "util.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

using intersection::make_err;
using intersection::make_ok;
using intersection::Result;

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id,
  [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
  const std::string & turn_direction, const bool has_traffic_light, rclcpp::Node & node,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  node_(node),
  lane_id_(lane_id),
  associative_ids_(associative_ids),
  turn_direction_(turn_direction),
  has_traffic_light_(has_traffic_light),
  occlusion_uuid_(tier4_autoware_utils::generateUUID())
{
  velocity_factor_.init(PlanningBehavior::INTERSECTION);
  planner_param_ = planner_param;

  {
    collision_state_machine_.setMarginTime(
      planner_param_.collision_detection.collision_detection_hold_time);
  }
  {
    before_creep_state_machine_.setMarginTime(
      planner_param_.occlusion.temporal_stop_time_before_peeking);
    before_creep_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    occlusion_stop_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    occlusion_stop_state_machine_.setState(StateMachine::State::GO);
  }
  {
    temporal_stop_before_attention_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    temporal_stop_before_attention_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    static_occlusion_timeout_state_machine_.setMarginTime(
      planner_param_.occlusion.static_occlusion_with_traffic_light_timeout);
    static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
  }

  decision_state_pub_ =
    node_.create_publisher<std_msgs::msg::String>("~/debug/intersection/decision_state", 1);
  ego_ttc_pub_ = node_.create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/intersection/ego_ttc", 1);
  object_ttc_pub_ = node_.create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/intersection/object_ttc", 1);
}

bool IntersectionModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::INTERSECTION);

  initializeRTCStatus();

  const auto decision_result = modifyPathVelocityDetail(path, stop_reason);
  prev_decision_result_ = decision_result;

  const std::string decision_type = "intersection" + std::to_string(module_id_) + " : " +
                                    intersection::formatDecisionResult(decision_result);
  std_msgs::msg::String decision_result_msg;
  decision_result_msg.data = decision_type;
  decision_state_pub_->publish(decision_result_msg);

  prepareRTCStatus(decision_result, *path);

  reactRTCApproval(decision_result, path, stop_reason);

  return true;
}

void IntersectionModule::initializeRTCStatus()
{
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
  // occlusion
  occlusion_safety_ = true;
  occlusion_stop_distance_ = std::numeric_limits<double>::lowest();
  occlusion_first_stop_required_ = false;
  // activated_ and occlusion_activated_ must be set from manager's RTC callback
}

intersection::DecisionResult IntersectionModule::modifyPathVelocityDetail(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  const auto traffic_prioritized_level = getTrafficPrioritizedLevel();
  const bool is_prioritized =
    traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED;

  const auto prepare_data = prepareIntersectionData(is_prioritized, path);
  if (!prepare_data) {
    return prepare_data.err();
  }
  const auto [interpolated_path_info, intersection_stoplines, path_lanelets] = prepare_data.ok();
  const auto & intersection_lanelets = intersection_lanelets_.value();

  // ==========================================================================================
  // stuck detection
  //
  // stuck vehicle detection is viable even if attention area is empty
  // so this needs to be checked before attention area validation
  // ==========================================================================================
  const auto is_stuck_status = isStuckStatus(*path, intersection_stoplines, path_lanelets);
  if (is_stuck_status) {
    return is_stuck_status.value();
  }

  // ==========================================================================================
  // basic data validation
  //
  // if attention area is empty, collision/occlusion detection is impossible
  //
  // if attention area is not null but default stop line is not available, ego/backward-path has
  // already passed the stop line, so ego is already in the middle of the intersection, or the
  // end of the ego path has just entered the entry of this intersection
  //
  // occlusion stop line is generated from the intersection of ego footprint along the path with the
  // attention area, so if this is null, ego has already passed the intersection, or the end of the
  // ego path has just entered the entry of this intersection
  // ==========================================================================================
  if (!intersection_lanelets.first_attention_area()) {
    return intersection::Indecisive{"attention area is empty"};
  }
  const auto first_attention_area = intersection_lanelets.first_attention_area().value();
  const auto default_stopline_idx_opt = intersection_stoplines.default_stopline;
  if (!default_stopline_idx_opt) {
    return intersection::Indecisive{"default stop line is null"};
  }
  const auto default_stopline_idx = default_stopline_idx_opt.value();
  const auto first_attention_stopline_idx_opt = intersection_stoplines.first_attention_stopline;
  const auto occlusion_peeking_stopline_idx_opt = intersection_stoplines.occlusion_peeking_stopline;
  if (!first_attention_stopline_idx_opt || !occlusion_peeking_stopline_idx_opt) {
    return intersection::Indecisive{"occlusion stop line is null"};
  }
  const auto first_attention_stopline_idx = first_attention_stopline_idx_opt.value();
  const auto occlusion_stopline_idx = occlusion_peeking_stopline_idx_opt.value();

  // ==========================================================================================
  // classify the objects to attention_area/intersection_area and update their position, velocity,
  // belonging attention lanelet, distance to corresponding stopline
  // ==========================================================================================
  updateObjectInfoManagerArea();

  // ==========================================================================================
  // occlusion_status is type of occlusion,
  // is_occlusion_cleared_with_margin indicates if occlusion is physically detected
  // is_occlusion_state indicates if occlusion is detected. OR occlusion is not detected but RTC for
  // intersection_occlusion is disapproved, which means ego is virtually occluded
  //
  // so is_occlusion_cleared_with_margin should be sent to RTC as module decision
  // and is_occlusion_status should be only used to decide ego action
  // !is_occlusion_state == !physically_occluded && !externally_occluded, so even if occlusion is
  // not detected but not approved, SAFE is not sent.
  // ==========================================================================================
  const auto [occlusion_status, is_occlusion_cleared_with_margin, is_occlusion_state] =
    getOcclusionStatus(traffic_prioritized_level, interpolated_path_info);

  const auto
    [is_over_1st_pass_judge_line, is_over_2nd_pass_judge_line, safely_passed_1st_judge_line,
     safely_passed_2nd_judge_line] =
      isOverPassJudgeLinesStatus(*path, is_occlusion_state, intersection_stoplines);

  // ==========================================================================================
  // calculate the expected vehicle speed and obtain the spatiotemporal profile of ego to the
  // exit of intersection
  // ==========================================================================================
  tier4_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array =
    calcIntersectionPassingTime(*path, is_prioritized, intersection_stoplines, &ego_ttc_time_array);

  updateObjectInfoManagerCollision(
    path_lanelets, time_distance_array, traffic_prioritized_level, safely_passed_1st_judge_line,
    safely_passed_2nd_judge_line);

  const auto [has_collision, collision_position] = detectCollision();
  if (is_permanent_go_) {
    if (has_collision) {
      // TODO(Mamoru Sobue): diagnosis
      return intersection::Indecisive{
        "ego is over the pass judge lines and collision is detected. need acceleration to keep "
        "safe."};
    }
    return intersection::Indecisive{"over pass judge lines and collision is not detected"};
  }
  /*
  const bool collision_on_1st_attention_lane =
    has_collision && collision_position == intersection::CollisionInterval::LanePosition::FIRST;
  if (
    is_over_1st_pass_judge_line && !is_over_2nd_pass_judge_line &&
    collision_on_1st_attention_lane) {
    // TODO(Mamoru Sobue): diagnosis
    return intersection::Indecisive{
      "ego is already over the 1st pass judge line although still before the 2nd pass judge line, "
      "but collision is detected on the first attention lane"};
  }
  */

  const auto closest_idx = intersection_stoplines.closest_idx;
  const bool is_over_default_stopline = util::isOverTargetIndex(
    *path, closest_idx, planner_data_->current_odometry->pose, default_stopline_idx);
  const auto collision_stopline_idx = is_over_default_stopline ? closest_idx : default_stopline_idx;

  // ==========================================================================================
  // pseudo collision detection on green light
  // ==========================================================================================
  const auto is_green_pseudo_collision_status =
    isGreenPseudoCollisionStatus(*path, collision_stopline_idx, intersection_stoplines);
  if (is_green_pseudo_collision_status) {
    return is_green_pseudo_collision_status.value();
  }

  // ==========================================================================================
  // yield stuck detection
  // ==========================================================================================
  const auto is_yield_stuck_status =
    isYieldStuckStatus(*path, interpolated_path_info, intersection_stoplines);
  if (is_yield_stuck_status) {
    return is_yield_stuck_status.value();
  }

  collision_state_machine_.setStateWithMarginTime(
    has_collision ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool has_collision_with_margin =
    collision_state_machine_.getState() == StateMachine::State::STOP;

  if (is_prioritized) {
    return intersection::FullyPrioritized{
      has_collision_with_margin, closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }

  // Safe
  if (!is_occlusion_state && !has_collision_with_margin) {
    return intersection::Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }
  // Only collision
  if (!is_occlusion_state && has_collision_with_margin) {
    return intersection::NonOccludedCollisionStop{
      closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }
  // Occluded
  // utility functions
  auto fromEgoDist = [&](const size_t index) {
    return motion_utils::calcSignedArcLength(path->points, closest_idx, index);
  };
  auto stoppedForDuration =
    [&](const size_t pos, const double duration, StateMachine & state_machine) {
      const double dist_stopline = fromEgoDist(pos);
      const bool approached_dist_stopline =
        (std::fabs(dist_stopline) < planner_param_.common.stopline_overshoot_margin);
      const bool over_stopline = (dist_stopline < 0.0);
      const bool is_stopped_duration = planner_data_->isVehicleStopped(duration);
      if (over_stopline) {
        state_machine.setState(StateMachine::State::GO);
      } else if (is_stopped_duration && approached_dist_stopline) {
        state_machine.setState(StateMachine::State::GO);
      }
      return state_machine.getState() == StateMachine::State::GO;
    };
  auto stoppedAtPosition = [&](const size_t pos, const double duration) {
    const double dist_stopline = fromEgoDist(pos);
    const bool approached_dist_stopline =
      (std::fabs(dist_stopline) < planner_param_.common.stopline_overshoot_margin);
    const bool over_stopline = (dist_stopline < -planner_param_.common.stopline_overshoot_margin);
    const bool is_stopped = planner_data_->isVehicleStopped(duration);
    if (over_stopline) {
      return true;
    } else if (is_stopped && approached_dist_stopline) {
      return true;
    }
    return false;
  };

  const auto occlusion_wo_tl_pass_judge_line_idx =
    intersection_stoplines.occlusion_wo_tl_pass_judge_line;
  const bool stopped_at_default_line = stoppedForDuration(
    default_stopline_idx, planner_param_.occlusion.temporal_stop_time_before_peeking,
    before_creep_state_machine_);
  if (stopped_at_default_line) {
    // ==========================================================================================
    // if specified the parameter occlusion.temporal_stop_before_attention_area OR
    // has_no_traffic_light_, ego will temporarily stop before entering attention area
    // ==========================================================================================
    const bool temporal_stop_before_attention_required =
      (planner_param_.occlusion.temporal_stop_before_attention_area || !has_traffic_light_)
        ? !stoppedForDuration(
            first_attention_stopline_idx,
            planner_param_.occlusion.temporal_stop_time_before_peeking,
            temporal_stop_before_attention_state_machine_)
        : false;
    if (!has_traffic_light_) {
      if (fromEgoDist(occlusion_wo_tl_pass_judge_line_idx) < 0) {
        return intersection::Indecisive{
          "already passed maximum peeking line in the absence of traffic light"};
      }
      return intersection::OccludedAbsenceTrafficLight{
        is_occlusion_cleared_with_margin,
        has_collision_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        first_attention_stopline_idx,
        occlusion_wo_tl_pass_judge_line_idx};
    }

    // ==========================================================================================
    // following remaining block is "has_traffic_light_"
    //
    // if ego is stuck by static occlusion in the presence of traffic light, start timeout count
    // ==========================================================================================
    const bool is_static_occlusion = occlusion_status == OcclusionType::STATICALLY_OCCLUDED;
    const bool is_stuck_by_static_occlusion =
      stoppedAtPosition(
        occlusion_stopline_idx, planner_param_.occlusion.temporal_stop_time_before_peeking) &&
      is_static_occlusion;
    if (has_collision_with_margin) {
      // if collision is detected, timeout is reset
      static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
    } else if (is_stuck_by_static_occlusion) {
      static_occlusion_timeout_state_machine_.setStateWithMarginTime(
        StateMachine::State::GO, logger_.get_child("static_occlusion"), *clock_);
    }
    const bool release_static_occlusion_stuck =
      (static_occlusion_timeout_state_machine_.getState() == StateMachine::State::GO);
    if (!has_collision_with_margin && release_static_occlusion_stuck) {
      return intersection::Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx};
    }
    // occlusion_status is either STATICALLY_OCCLUDED or DYNAMICALLY_OCCLUDED
    const double max_timeout =
      planner_param_.occlusion.static_occlusion_with_traffic_light_timeout +
      planner_param_.occlusion.occlusion_detection_hold_time;
    const std::optional<double> static_occlusion_timeout =
      is_stuck_by_static_occlusion
        ? std::make_optional<double>(
            max_timeout - static_occlusion_timeout_state_machine_.getDuration() -
            occlusion_stop_state_machine_.getDuration())
        : (is_static_occlusion ? std::make_optional<double>(max_timeout) : std::nullopt);
    if (has_collision_with_margin) {
      return intersection::OccludedCollisionStop{
        is_occlusion_cleared_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        collision_stopline_idx,
        first_attention_stopline_idx,
        occlusion_stopline_idx,
        static_occlusion_timeout};
    } else {
      return intersection::PeekingTowardOcclusion{
        is_occlusion_cleared_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        collision_stopline_idx,
        first_attention_stopline_idx,
        occlusion_stopline_idx,
        static_occlusion_timeout};
    }
  } else {
    const auto occlusion_stopline =
      (planner_param_.occlusion.temporal_stop_before_attention_area || !has_traffic_light_)
        ? first_attention_stopline_idx
        : occlusion_stopline_idx;
    return intersection::FirstWaitBeforeOcclusion{
      is_occlusion_cleared_with_margin, closest_idx, default_stopline_idx, occlusion_stopline};
  }
}

// template-specification based visitor pattern
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct VisitorSwitch : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
VisitorSwitch(Ts...) -> VisitorSwitch<Ts...>;

template <typename T>
void prepareRTCByDecisionResult(
  const T & result, const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  double * occlusion_distance)
{
  static_assert("Unsupported type passed to prepareRTCByDecisionResult");
  return;
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const intersection::Indecisive & result,
  [[maybe_unused]] const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::StuckStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "StuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
  if (result.occlusion_stopline_idx) {
    const auto occlusion_stopline_idx = result.occlusion_stopline_idx.value();
    *occlusion_distance =
      motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  }
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::YieldStuckStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "YieldStuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::NonOccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "NonOccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  const auto occlusion_stopline = result.occlusion_stopline_idx;
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::FirstWaitBeforeOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FirstWaitBeforeOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto first_stopline_idx = result.first_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, first_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::PeekingTowardOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "PeekingTowardOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::OccludedAbsenceTrafficLight & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedAbsenceTrafficLight");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.closest_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance = 0;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::OccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::Safe & result, const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "Safe");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const intersection::FullyPrioritized & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FullyPrioritized");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

void IntersectionModule::prepareRTCStatus(
  const intersection::DecisionResult & decision_result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  bool default_safety = true;
  double default_distance = std::numeric_limits<double>::lowest();
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      prepareRTCByDecisionResult(
        decision, path, &default_safety, &default_distance, &occlusion_safety_,
        &occlusion_stop_distance_);
    }},
    decision_result);
  setSafe(default_safety);
  setDistance(default_distance);
  occlusion_first_stop_required_ =
    std::holds_alternative<intersection::FirstWaitBeforeOcclusion>(decision_result);
}

template <typename T>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved, const T & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, IntersectionModule::DebugData * debug_data)
{
  static_assert("Unsupported type passed to reactRTCByDecisionResult");
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const bool rtc_occlusion_approved,
  [[maybe_unused]] const intersection::Indecisive & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] StopReason * stop_reason,
  [[maybe_unused]] VelocityFactorInterface * velocity_factor,
  [[maybe_unused]] IntersectionModule::DebugData * debug_data)
{
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::StuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "StuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data->conflicting_targets);
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (
    !rtc_occlusion_approved && decision_result.occlusion_stopline_idx &&
    planner_param.occlusion.enable) {
    const auto occlusion_stopline_idx = decision_result.occlusion_stopline_idx.value();
    planning_utils::setVelocityFromIndex(occlusion_stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(occlusion_stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::YieldStuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "YieldStuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data->conflicting_targets);
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::NonOccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "NonOccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::FirstWaitBeforeOcclusion & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FirstWaitBeforeOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.first_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_first_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    if (planner_param.occlusion.creep_during_peeking.enable) {
      const size_t occlusion_peeking_stopline = decision_result.occlusion_stopline_idx;
      const size_t closest_idx = decision_result.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stopline; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.creep_during_peeking.creep_velocity, path);
      }
    }
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::PeekingTowardOcclusion & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "PeekingTowardOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  // NOTE: creep_velocity should be inserted first at closest_idx if !rtc_default_approved
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const size_t occlusion_peeking_stopline =
      decision_result.temporal_stop_before_attention_required
        ? decision_result.first_attention_stopline_idx
        : decision_result.occlusion_stopline_idx;
    if (planner_param.occlusion.creep_during_peeking.enable) {
      const size_t closest_idx = decision_result.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stopline; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.creep_during_peeking.creep_velocity, path);
      }
    }
    planning_utils::setVelocityFromIndex(occlusion_peeking_stopline, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_peeking_stopline, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_peeking_stopline).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(occlusion_peeking_stopline).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::OccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.temporal_stop_before_attention_required
                                ? decision_result.first_attention_stopline_idx
                                : decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::OccludedAbsenceTrafficLight & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedAbsenceTrafficLight, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.closest_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && decision_result.temporal_stop_before_attention_required) {
    const auto stopline_idx = decision_result.first_attention_area_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && !decision_result.temporal_stop_before_attention_required) {
    const auto closest_idx = decision_result.closest_idx;
    const auto peeking_limit_line = decision_result.peeking_limit_line_idx;
    for (auto i = closest_idx; i <= peeking_limit_line; ++i) {
      planning_utils::setVelocityFromIndex(
        i, planner_param.occlusion.creep_velocity_without_traffic_light, path);
    }
    debug_data->absence_traffic_light_creep_wall =
      planning_utils::getAheadPose(closest_idx, baselink2front, *path);
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::Safe & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "Safe, approval = (default: %d, occlusion: %d)", rtc_default_approved, rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const intersection::FullyPrioritized & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor,
  IntersectionModule::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FullyPrioritized, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

void IntersectionModule::reactRTCApproval(
  const intersection::DecisionResult & decision_result,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason)
{
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      reactRTCApprovalByDecisionResult(
        activated_, occlusion_activated_, decision, planner_param_, baselink2front, path,
        stop_reason, &velocity_factor_, &debug_data_);
    }},
    decision_result);
  return;
}

bool IntersectionModule::isGreenSolidOn() const
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & lane = lanelet_map_ptr->laneletLayer.get(lane_id_);

  std::optional<lanelet::Id> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return false;
  }
  const auto tl_info_opt = planner_data_->getTrafficSignal(
    tl_id.value(), true /* traffic light module keeps last observation*/);
  if (!tl_info_opt) {
    // the info of this traffic light is not available
    return false;
  }
  const auto & tl_info = tl_info_opt.value();
  for (auto && tl_light : tl_info.signal.elements) {
    if (
      tl_light.color == TrafficSignalElement::GREEN &&
      tl_light.shape == TrafficSignalElement::CIRCLE) {
      return true;
    }
  }
  return false;
}

IntersectionModule::TrafficPrioritizedLevel IntersectionModule::getTrafficPrioritizedLevel() const
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;

  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & lane = lanelet_map_ptr->laneletLayer.get(lane_id_);

  std::optional<lanelet::Id> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto tl_info_opt = planner_data_->getTrafficSignal(
    tl_id.value(), true /* traffic light module keeps last observation*/);
  if (!tl_info_opt) {
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto & tl_info = tl_info_opt.value();
  bool has_amber_signal{false};
  for (auto && tl_light : tl_info.signal.elements) {
    if (tl_light.color == TrafficSignalElement::AMBER) {
      has_amber_signal = true;
    }
    if (tl_light.color == TrafficSignalElement::RED) {
      // NOTE: Return here since the red signal has the highest priority.
      return TrafficPrioritizedLevel::FULLY_PRIORITIZED;
    }
  }
  if (has_amber_signal) {
    return TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED;
  }
  return TrafficPrioritizedLevel::NOT_PRIORITIZED;
}

IntersectionModule::PassJudgeStatus IntersectionModule::isOverPassJudgeLinesStatus(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const bool is_occlusion_state,
  const intersection::IntersectionStopLines & intersection_stoplines)
{
  const auto & current_pose = planner_data_->current_odometry->pose;
  const auto closest_idx = intersection_stoplines.closest_idx;
  const auto default_stopline_idx = intersection_stoplines.default_stopline.value();
  const auto first_pass_judge_line_idx = intersection_stoplines.first_pass_judge_line;
  const auto occlusion_wo_tl_pass_judge_line_idx =
    intersection_stoplines.occlusion_wo_tl_pass_judge_line;
  const auto occlusion_stopline_idx = intersection_stoplines.occlusion_peeking_stopline.value();
  const size_t pass_judge_line_idx = [&]() {
    if (planner_param_.occlusion.enable) {
      if (has_traffic_light_) {
        // ==========================================================================================
        // if ego passed the first_pass_judge_line while it is peeking to occlusion, then its
        // position is changed to occlusion_stopline_idx. even if occlusion is cleared by peeking,
        // its position should be occlusion_stopline_idx as before
        // ==========================================================================================
        if (passed_1st_judge_line_while_peeking_) {
          return occlusion_stopline_idx;
        }
        const bool is_over_first_pass_judge_line =
          util::isOverTargetIndex(path, closest_idx, current_pose, first_pass_judge_line_idx);
        if (is_occlusion_state && is_over_first_pass_judge_line) {
          passed_1st_judge_line_while_peeking_ = true;
          return occlusion_stopline_idx;
        }
        // ==========================================================================================
        // Otherwise it is first_pass_judge_line
        // ==========================================================================================
        return first_pass_judge_line_idx;
      } else if (is_occlusion_state) {
        // ==========================================================================================
        // if there is no traffic light and occlusion is detected, pass_judge position is beyond
        // the boundary of first attention area
        // ==========================================================================================
        return occlusion_wo_tl_pass_judge_line_idx;
      } else {
        // ==========================================================================================
        // if there is no traffic light and occlusion is not detected, pass_judge position is
        // default position
        // ==========================================================================================
        return first_pass_judge_line_idx;
      }
    }
    return first_pass_judge_line_idx;
  }();

  const bool was_safe = std::holds_alternative<intersection::Safe>(prev_decision_result_);

  const bool is_over_1st_pass_judge_line =
    util::isOverTargetIndex(path, closest_idx, current_pose, pass_judge_line_idx);
  bool safely_passed_1st_judge_line_first_time = false;
  if (is_over_1st_pass_judge_line && was_safe && !safely_passed_1st_judge_line_time_) {
    safely_passed_1st_judge_line_time_ = clock_->now();
    safely_passed_1st_judge_line_first_time = true;
  }
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_.first_pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, path);
  debug_data_.passed_first_pass_judge = safely_passed_1st_judge_line_time_.has_value();
  const auto second_pass_judge_line_idx = intersection_stoplines.second_pass_judge_line;
  const bool is_over_2nd_pass_judge_line =
    util::isOverTargetIndex(path, closest_idx, current_pose, second_pass_judge_line_idx);
  bool safely_passed_2nd_judge_line_first_time = false;
  if (is_over_2nd_pass_judge_line && was_safe && !safely_passed_2nd_judge_line_time_) {
    safely_passed_2nd_judge_line_time_ = clock_->now();
    safely_passed_2nd_judge_line_first_time = true;
  }
  debug_data_.second_pass_judge_wall_pose =
    planning_utils::getAheadPose(second_pass_judge_line_idx, baselink2front, path);
  debug_data_.passed_second_pass_judge = safely_passed_2nd_judge_line_time_.has_value();

  const bool is_over_default_stopline =
    util::isOverTargetIndex(path, closest_idx, current_pose, default_stopline_idx);
  if (
    ((is_over_default_stopline ||
      planner_param_.common.enable_pass_judge_before_default_stopline) &&
     is_over_2nd_pass_judge_line && was_safe) ||
    is_permanent_go_) {
    // ==========================================================================================
    // this body is active if ego is
    // - over the default stopline AND
    // - over the 1st && 2nd pass judge line AND
    // - previously safe
    // ,
    // which means ego can stop even if it is over the 1st pass judge line but
    // - before default stopline OR
    // - before the 2nd pass judge line OR
    // - or previously unsafe
    // .
    //
    // in order for ego to continue peeking or collision detection when occlusion is detected after
    // ego passed the 1st pass judge line, it needs to be
    // - before the default stopline OR
    // - before the 2nd pass judge line OR
    // - previously unsafe
    // ==========================================================================================
    is_permanent_go_ = true;
  }
  return {
    is_over_1st_pass_judge_line, is_over_2nd_pass_judge_line,
    safely_passed_1st_judge_line_first_time, safely_passed_2nd_judge_line_first_time};
}

}  // namespace behavior_velocity_planner
