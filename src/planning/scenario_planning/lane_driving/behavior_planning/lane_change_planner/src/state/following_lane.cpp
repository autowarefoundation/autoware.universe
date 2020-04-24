/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/state/common_functions.h>
#include <lane_change_planner/state/following_lane.h>
#include <lane_change_planner/utilities.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>

namespace lane_change_planner
{
FollowingLaneState::FollowingLaneState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}

State FollowingLaneState::getCurrentState() const { return State::FOLLOWING_LANE; }

void FollowingLaneState::entry()
{
  ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();
  lane_change_approved_ = false;
  force_lane_change_ = false;
  status_.lane_change_available = false;
  status_.lane_change_ready = false;
}

autoware_planning_msgs::PathWithLaneId FollowingLaneState::getPath() const
{
  return status_.lane_follow_path;
}

void FollowingLaneState::update()
{
  // update input data
  current_twist_ = data_manager_ptr_->getCurrentSelfVelocity();
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();
  dynamic_objects_ = data_manager_ptr_->getDynamicObjects();
  lane_change_approved_ = data_manager_ptr_->getLaneChangeApproval();
  force_lane_change_ = data_manager_ptr_->getForceLaneChangeSignal();

  lanelet::ConstLanelet current_lane;
  const double backward_path_length = ros_parameters_.backward_path_length;
  const double forward_path_length = ros_parameters_.forward_path_length;
  // update lanes
  {
    if (!route_handler_ptr_->getClosestLaneletWithinRoute(current_pose_.pose, &current_lane)) {
      ROS_ERROR("failed to find closest lanelet within route!!!");
      return;
    }
    current_lanes_ = route_handler_ptr_->getLaneletSequence(
      current_lane, current_pose_.pose, backward_path_length, forward_path_length);
    lanelet::ConstLanelet lane_change_lane;
    if (route_handler_ptr_->getLaneChangeTarget(current_lane, &lane_change_lane)) {
      constexpr double lane_change_lane_length = 100.0;
      lane_change_lanes_ = route_handler_ptr_->getLaneletSequence(
        lane_change_lane, current_pose_.pose, lane_change_lane_length, lane_change_lane_length);
    } else {
      lane_change_lanes_.clear();
    }
  }
  // update lane_follow_path
  {
    const double lane_change_prepare_duration = ros_parameters_.lane_change_prepare_duration;
    const double lane_changing_duration = ros_parameters_.lane_changing_duration;
    const double minimum_lane_change_length = ros_parameters_.minimum_lane_change_length;
    status_.lane_follow_path = route_handler_ptr_->getReferencePath(
      current_lanes_, current_pose_.pose, backward_path_length, forward_path_length,
      minimum_lane_change_length);

    if (!lane_change_lanes_.empty()) {
      status_.lane_change_path = route_handler_ptr_->getLaneChangePath(
        current_lanes_, lane_change_lanes_, current_pose_.pose, current_twist_->twist,
        backward_path_length, forward_path_length, lane_change_prepare_duration,
        lane_changing_duration, minimum_lane_change_length);
    }
    status_.lane_follow_lane_ids = util::getIds(current_lanes_);
    status_.lane_change_lane_ids = util::getIds(lane_change_lanes_);
  }

  // update drivable area
  {
    const double width = ros_parameters_.drivable_area_width;
    const double height = ros_parameters_.drivable_area_height;
    const double resolution = ros_parameters_.drivable_area_resolution;
    status_.lane_follow_path.drivable_area =
      util::convertLanesToDrivableArea(current_lanes_, current_pose_, width, height, resolution);
  }

  // update lane_change_ready flags
  {
    status_.lane_change_ready = false;
    status_.lane_change_available = false;

    if (!lane_change_lanes_.empty()) {
      status_.lane_change_available = true;
      if (hasEnoughDistance() && isLaneChangePathSafe() && !isLaneBlocked(lane_change_lanes_)) {
        status_.lane_change_ready = true;
      }
    }
  }
}

State FollowingLaneState::getNextState() const
{
  if (current_lanes_.empty()) {
    ROS_ERROR_THROTTLE(1, "current lanes empty. Keeping state.");
    return State::FOLLOWING_LANE;
  }
  if (route_handler_ptr_->isInPreferredLane(current_pose_) && isLaneBlocked(current_lanes_)) {
    return State::BLOCKED_BY_OBSTACLE;
  }
  if (isLaneChangeAvailable() && laneChangeForcedByOperator()) {
    return State::FORCING_LANE_CHANGE;
  }
  if (isLaneChangeReady() && isLaneChangeApproved()) {
    return State::EXECUTING_LANE_CHANGE;
  }
  return State::FOLLOWING_LANE;
}

bool FollowingLaneState::isLaneBlocked(const lanelet::ConstLanelets & lanes) const
{
  const auto arc = lanelet::utils::getArcCoordinates(lanes, current_pose_.pose);
  constexpr double max_check_distance = 100;
  double static_obj_velocity_thresh = ros_parameters_.static_obstacle_velocity_thresh;
  const double lane_changeable_distance_left =
    route_handler_ptr_->getLaneChangeableDistance(current_pose_.pose, LaneChangeDirection::LEFT);
  const double lane_changeable_distance_right =
    route_handler_ptr_->getLaneChangeableDistance(current_pose_.pose, LaneChangeDirection::RIGHT);
  const double lane_changeable_distance =
    std::max(lane_changeable_distance_left, lane_changeable_distance_right);
  const double check_distance = std::min(max_check_distance, lane_changeable_distance);
  const auto polygon =
    lanelet::utils::getPolygonFromArcLength(lanes, arc.length, arc.length + check_distance);

  if (polygon.size() < 3) {
    ROS_WARN_STREAM(
      "could not get polygon from lanelet with arc lengths: " << arc.length << " to "
                                                              << arc.length + check_distance);
    return false;
  }

  for (const auto & obj : dynamic_objects_->objects) {
    const auto velocity = util::l2Norm(obj.state.twist_covariance.twist.linear);
    if (velocity < static_obj_velocity_thresh) {
      const auto position =
        lanelet::utils::conversion::toLaneletPoint(obj.state.pose_covariance.pose.position);
      const auto distance = boost::geometry::distance(
        lanelet::utils::to2D(position).basicPoint(), lanelet::utils::to2D(polygon).basicPolygon());
      if (distance < std::numeric_limits<double>::epsilon()) {
        return true;
      }
    }
  }
  return false;
}

bool FollowingLaneState::isVehicleInPreferredLane() const
{
  return route_handler_ptr_->isInPreferredLane(current_pose_);
}

bool FollowingLaneState::isTooCloseToDeadEnd() const { return false; }

bool FollowingLaneState::isLaneChangeApproved() const { return lane_change_approved_; }

bool FollowingLaneState::laneChangeForcedByOperator() const { return force_lane_change_; }

bool FollowingLaneState::isLaneChangeReady() const { return status_.lane_change_ready; }

bool FollowingLaneState::isLaneChangeAvailable() const { return status_.lane_change_available; }

bool FollowingLaneState::hasEnoughDistance() const
{
  const double lane_change_prepare_duration = ros_parameters_.lane_change_prepare_duration;
  const double lane_changing_duration = ros_parameters_.lane_changing_duration;
  const double lane_change_total_duration = lane_change_prepare_duration + lane_changing_duration;
  const double vehicle_speed = util::l2Norm(current_twist_->twist.linear);
  const double lane_change_total_distance = lane_change_total_duration * vehicle_speed;

  if (
    lane_change_total_distance > util::getDistanceToEndOfLane(current_pose_.pose, current_lanes_)) {
    return false;
  }

  if (
    lane_change_prepare_duration * vehicle_speed >
    util::getDistanceToNextIntersection(current_pose_.pose, current_lanes_)) {
    return false;
  }
  return true;
}

bool FollowingLaneState::isLaneChangePathSafe() const
{
  return state_machine::common_functions::isLaneChangePathSafe(
    status_.lane_change_path, current_lanes_, lane_change_lanes_, dynamic_objects_,
    current_pose_.pose, current_twist_->twist, ros_parameters_, true);
}

}  // namespace lane_change_planner
