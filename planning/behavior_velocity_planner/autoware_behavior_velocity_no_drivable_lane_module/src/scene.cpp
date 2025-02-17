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

#include "scene.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::createPoint;

NoDrivableLaneModule::NoDrivableLaneModule(
  const int64_t module_id, const int64_t lane_id, const PlannerParam & planner_param,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  planner_param_(planner_param),
  debug_data_(),
  state_(State::INIT)
{
}

bool NoDrivableLaneModule::modifyPathVelocity(PathWithLaneId * path)
{
  if (path->points.empty()) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & no_drivable_lane = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto & no_drivable_lane_polygon =
    lanelet::utils::to2D(no_drivable_lane).polygon2d().basicPolygon();

  path_no_drivable_lane_polygon_intersection =
    getPathIntersectionWithNoDrivableLanePolygon(*path, no_drivable_lane_polygon, ego_pos, 2);

  distance_ego_first_intersection = 0.0;

  if (path_no_drivable_lane_polygon_intersection.first_intersection_point) {
    first_intersection_point =
      path_no_drivable_lane_polygon_intersection.first_intersection_point.value();
    distance_ego_first_intersection = autoware::motion_utils::calcSignedArcLength(
      path->points, planner_data_->current_odometry->pose.position, first_intersection_point);
    distance_ego_first_intersection -= planner_data_->vehicle_info_.max_longitudinal_offset_m;
  }

  initialize_debug_data(no_drivable_lane, ego_pos);

  switch (state_) {
    case State::INIT: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Init");
      }

      handle_init_state();

      break;
    }

    case State::APPROACHING: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Approaching ");
      }

      handle_approaching_state(path);

      break;
    }

    case State::INSIDE_NO_DRIVABLE_LANE: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "INSIDE_NO_DRIVABLE_LANE");
      }

      handle_inside_no_drivable_lane_state(path);

      break;
    }

    case State::STOPPED: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "STOPPED");
      }

      handle_stopped_state(path);

      break;
    }

    default: {
      RCLCPP_ERROR(logger_, "ERROR. Undefined case");
      return false;
    }
  }
  return true;
}

void NoDrivableLaneModule::handle_init_state()
{
  if (
    (path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon) ||
    ((path_no_drivable_lane_polygon_intersection.first_intersection_point) &&
     (distance_ego_first_intersection <= planner_param_.stop_margin))) {
    state_ = State::INSIDE_NO_DRIVABLE_LANE;
  } else if (
    (path_no_drivable_lane_polygon_intersection.first_intersection_point) &&
    (distance_ego_first_intersection > planner_param_.stop_margin)) {
    state_ = State::APPROACHING;
  } else {
    state_ = State::INIT;
  }
}

void NoDrivableLaneModule::handle_approaching_state(PathWithLaneId * path)
{
  const double longitudinal_offset =
    -1.0 * (planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m);

  const auto op_target_point = autoware::motion_utils::calcLongitudinalOffsetPoint(
    path->points, first_intersection_point, longitudinal_offset);

  geometry_msgs::msg::Point target_point;

  if (op_target_point) {
    target_point = op_target_point.value();
  }

  const auto target_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(path->points, target_point);

  const auto & op_target_point_idx =
    autoware::motion_utils::insertTargetPoint(target_segment_idx, target_point, path->points, 5e-2);
  size_t target_point_idx = 0;
  if (op_target_point_idx) {
    target_point_idx = op_target_point_idx.value();
  }

  geometry_msgs::msg::Point stop_point =
    autoware::universe_utils::getPoint(path->points.at(target_point_idx).point);

  const auto & op_stop_pose =
    planning_utils::insertStopPoint(stop_point, target_segment_idx, *path);

  // Get stop point and stop factor
  {
    const auto & stop_pose = op_stop_pose.value();
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");

    const auto virtual_wall_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      path->points, stop_pose.position, debug_data_.base_link2front);

    debug_data_.stop_pose = virtual_wall_pose.value();
  }

  const size_t current_seg_idx = findEgoSegmentIndex(path->points);
  const auto intersection_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(path->points, first_intersection_point);
  const double signed_arc_dist_to_intersection_point =
    autoware::motion_utils::calcSignedArcLength(
      path->points, planner_data_->current_odometry->pose.position, current_seg_idx,
      first_intersection_point, intersection_segment_idx) -
    planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // Move to stopped state if stopped
  if (
    (signed_arc_dist_to_intersection_point <= planner_param_.stop_margin) &&
    (planner_data_->isVehicleStopped())) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
      RCLCPP_INFO_STREAM(
        logger_, "signed_arc_dist_to_stop_point = " << signed_arc_dist_to_intersection_point);
    }

    if (signed_arc_dist_to_intersection_point < 0.0) {
      RCLCPP_ERROR(
        logger_, "Failed to stop before no drivable lane but ego stopped. Change state to STOPPED");
    }

    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_inside_no_drivable_lane_state(PathWithLaneId * path)
{
  const auto & current_point = planner_data_->current_odometry->pose.position;
  const size_t current_seg_idx = findEgoSegmentIndex(path->points);

  // Insert stop point
  planning_utils::insertStopPoint(current_point, current_seg_idx, *path);

  // Get stop point and stop factor
  {
    const auto & stop_pose = autoware::universe_utils::getPose(path->points.at(0));
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");

    const auto & virtual_wall_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      path->points, stop_pose.position, debug_data_.base_link2front);

    debug_data_.stop_pose = virtual_wall_pose.value();
  }

  // Move to stopped state if stopped
  if (planner_data_->isVehicleStopped()) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
    }
    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_stopped_state(PathWithLaneId * path)
{
  const auto & stopped_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    path->points, planner_data_->current_odometry->pose.position, 0.0);

  if (!stopped_pose) {
    state_ = State::INIT;
    return;
  }

  SegmentIndexWithPose ego_pos_on_path;
  ego_pos_on_path.pose = stopped_pose.value();
  ego_pos_on_path.index = findEgoSegmentIndex(path->points);

  // Insert stop pose
  planning_utils::insertStopPoint(ego_pos_on_path.pose.position, ego_pos_on_path.index, *path);

  // Get stop point and stop factor
  {
    const auto & stop_pose = ego_pos_on_path.pose;
    planning_factor_interface_->add(
      path->points, planner_data_->current_odometry->pose, stop_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP, tier4_planning_msgs::msg::SafetyFactorArray{},
      true /*is_driving_forward*/, 0.0, 0.0 /*shift distance*/, "");

    const auto virtual_wall_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      path->points, stop_pose.position, debug_data_.base_link2front);

    debug_data_.stop_pose = virtual_wall_pose.value();
  }
}

void NoDrivableLaneModule::initialize_debug_data(
  const lanelet::Lanelet & no_drivable_lane, const geometry_msgs::msg::Point & ego_pos)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_.path_polygon_intersection = path_no_drivable_lane_polygon_intersection;

  for (const auto & p : no_drivable_lane.polygon2d().basicPolygon()) {
    debug_data_.no_drivable_lane_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }
}

}  // namespace autoware::behavior_velocity_planner
