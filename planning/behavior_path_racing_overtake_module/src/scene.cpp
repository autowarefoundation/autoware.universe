// Copyright 2021 TIER IV, Inc.
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

#include "behavior_path_racing_overtake_module/scene.hpp"

#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "motion_utils/trajectory/interpolation.hpp"

#include <algorithm>
#include <memory>
#include <string>
// #include <functional>

namespace behavior_path_planner
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using motion_utils::calcInterpolatedPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getPoint;

/**
 * @brief Add lateral offset to the path
 * @param path input path
 * @param lateral_offset lateral offset to be added
 * @return PathWithLaneId path with lateral offset
 */
PathWithLaneId addLateralOffset(const PathWithLaneId & path, const double & lateral_offset)
{
  PathWithLaneId path_with_lateral_offset = path;
  for (auto & point : path_with_lateral_offset.points) {
    double yaw = tf2::getYaw(point.point.pose.orientation);
    point.point.pose.position.x -= lateral_offset * std::sin(yaw);
    point.point.pose.position.y += lateral_offset * std::cos(yaw);
  }
  return path_with_lateral_offset;
}

struct RivalVehicle
{
  PredictedObject object;
  double longitudinal_from_ego;
  double lateral_from_ego;
};

/**
 * @brief Detect rival vehicle in ego course
 * @param ego_pose ego pose
 * @param path path
 * @param objects predicted objects
 * @return std::optional<RivalVehicle> rival vehicle
 */
std::optional<RivalVehicle> detectRivalVehicleInEgoCourse(
  const Pose & ego_pose, const PathWithLaneId & path, const std::vector<PredictedObject> & objects)
{
  auto self_odom_frenet = utils::convertToFrenetPoint(path.points, ego_pose.position, 0);

  std::optional<PredictedObject> closest_front_object_ = std::nullopt;
  double closest_front_object_length = std::numeric_limits<double>::max();
  double closest_front_object_distance;

  for (const auto & object : objects) {
    auto obj_frenet = utils::convertToFrenetPoint(
      path.points, object.kinematics.initial_pose_with_covariance.pose.position, 0);

    if (
      obj_frenet.length - self_odom_frenet.length > 0.0 &&
      obj_frenet.length - self_odom_frenet.length < closest_front_object_length) {
      closest_front_object_ = object;
      closest_front_object_length = obj_frenet.length - self_odom_frenet.length;
      closest_front_object_distance = obj_frenet.distance - self_odom_frenet.distance;
    }
  }

  if (!closest_front_object_.has_value()) {
    return std::nullopt;
  }

  RivalVehicle rival_vehicle;
  rival_vehicle.object = closest_front_object_.value();
  rival_vehicle.longitudinal_from_ego = closest_front_object_length;
  rival_vehicle.lateral_from_ego = closest_front_object_distance;
  return rival_vehicle;
}

namespace racing_overtake_state
{

void RacingOverTakeState::setContext(Context * context)
{
  context_ = context;
}

void ModuleNotLaunched::update(PlannerDataPtr planner_data)
{
  auto center_path = *utils::generateCenterLinePath(planner_data);
  auto rival_vehicle = detectRivalVehicleInEgoCourse(
    planner_data->self_odometry->pose.pose, center_path, planner_data->dynamic_object->objects);
  if (!rival_vehicle.has_value()) {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 5.0) {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 20.0) {
    context_->transitionTo(std::make_unique<Approach>());
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 50.0) {
    context_->transitionTo(std::make_unique<Overtaking>());
    return;
  }
}

void Approach::update(PlannerDataPtr planner_data)
{
  auto center_path = *utils::generateCenterLinePath(planner_data);
  auto rival_vehicle = detectRivalVehicleInEgoCourse(
    planner_data->self_odometry->pose.pose, center_path, planner_data->dynamic_object->objects);
  if (!rival_vehicle.has_value()) {
    context_->transitionTo(std::make_unique<ModuleNotLaunched>());
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 5.0) {
    context_->transitionTo(std::make_unique<ModuleNotLaunched>());
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 20.0) {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < 50.0) {
    context_->transitionTo(std::make_unique<Overtaking>());
    return;
  }
}

void Overtaking::update(PlannerDataPtr)
{
}

// class AfterOvertake : public RacingOverTakeState
// {
// };

Context::Context(std::unique_ptr<RacingOverTakeState> state) : state_(std::move(state))
{
  state_->setContext(this);
}
void Context::update(PlannerDataPtr planner_data)
{
  state_->update(planner_data);
}
void Context::transitionTo(std::unique_ptr<RacingOverTakeState> state)
{
  state_ = std::move(state);
  state_->setContext(this);
}

std::unique_ptr<RacingOverTakeState> Context::getState()
{
  return std::move(state_);
}

}  // namespace racing_overtake_state

RacingOvertakeModule::RacingOvertakeModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<RacingOvertakeParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},
  parameters_{parameters}
{
}

void RacingOvertakeModule::processOnEntry()
{
}

void RacingOvertakeModule::processOnExit()
{
}

bool RacingOvertakeModule::isExecutionRequested() const
{
  // return OverTakeState::DEFAULT != overtake_state_;
  return true;
}

bool RacingOvertakeModule::isExecutionReady() const
{
  return true;
}

bool RacingOvertakeModule::isReadyForNextRequest(
  const double & min_request_time_sec, bool override_requests) const noexcept
{
  rclcpp::Time current_time = clock_->now();
  const auto interval_from_last_request_sec = current_time - last_requested_shift_change_time_;

  if (interval_from_last_request_sec.seconds() >= min_request_time_sec && !override_requests) {
    last_requested_shift_change_time_ = current_time;
    return true;
  }

  return false;
}

bool RacingOvertakeModule::canTransitSuccessState()
{
  // return false;  // TODO: implement
  // if (overtake_state_ == OverTakeState::FINISH)
  // {
  //   return true;
  // }
  return false;
}

void RacingOvertakeModule::updateData()
{
  // context_.update(planner_data_);
  // std::cerr << "Curren State: " << context_.getState()->getName() << std::endl;
}

BehaviorModuleOutput RacingOvertakeModule::plan()
{
  BehaviorModuleOutput output = getPreviousModuleOutput();
  return output;
}

ShiftLine RacingOvertakeModule::getOverTakeShiftLine(
  const PathWithLaneId & path, const PredictedObject & object) const
{
  auto obj_frenet = utils::convertToFrenetPoint(
    path.points, object.kinematics.initial_pose_with_covariance.pose.position, 0);
  auto front_of_object = calcInterpolatedPose(path.points, obj_frenet.length);
  auto backward_of_object = calcInterpolatedPose(path.points, obj_frenet.length - 20.0);
  double shift_length_candidate1 = obj_frenet.distance + 4.0;
  double shift_length_candidate2 = obj_frenet.distance - 4.0;
  double shift_length = std::abs(shift_length_candidate1) < std::abs(shift_length_candidate2)
                          ? shift_length_candidate1
                          : shift_length_candidate2;
  ShiftLine shift_line;
  shift_line.start = backward_of_object;
  shift_line.end = front_of_object;
  shift_line.end_shift_length = shift_length;
  return shift_line;
}

CandidateOutput RacingOvertakeModule::planCandidate() const
{
  // PathShifter path_shifter_local = path_shifter_;
  // ShiftedPath shifted_path;
  // path_shifter_local.generate(&shifted_path);
  CandidateOutput candidate_output;
  // candidate_output.path_candidate = *planner_data_->prev_output_path;
  candidate_output.path_candidate = center_path_;
  return candidate_output;
}

BehaviorModuleOutput RacingOvertakeModule::planWaitingApproval()
{
  // BehaviorModuleOutput output;
  // output.path = center_path_;
  // output.reference_path = center_path_;
  // return output;
  return getPreviousModuleOutput();
}
}  // namespace behavior_path_planner
