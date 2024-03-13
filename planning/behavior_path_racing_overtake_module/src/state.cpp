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

#include "behavior_path_racing_overtake_module/state.hpp"

#include "behavior_path_planner_common/utils/utils.hpp"
#include "behavior_path_racing_overtake_module/util.hpp"

namespace behavior_path_planner::racing_overtake::state
{

using behavior_path_planner::utils::convertToFrenetPoint;
using behavior_path_planner::utils::generateCenterLinePath;

void RacingOverTakeState::setContext(Context* context)
{
  context_ = context;
}

void ModuleNotLaunched::update(PlannerDataPtr planner_data)
{
  auto center_path = *generateCenterLinePath(planner_data);
  auto rival_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                           planner_data->dynamic_object->objects,
                                                           context_->getParameters().ego_course_width);

  if (!rival_vehicle.has_value())
  {
    return;
  }

  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().too_close_to_overtake_distance)
  {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().start_overtake_distance)
  {
    context_->transitionTo<Overtaking>();
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().prepare_overtake_distance)
  {
    context_->transitionTo<Approach>();
    return;
  }
  return;
}

std::string ModuleNotLaunched::getName() const
{
  return "ModuleNotLaunched";
}

void ModuleNotLaunched::getPath(PlannerDataPtr, BehaviorModuleOutput*)
{
  return;
}

void Approach::update(PlannerDataPtr planner_data)
{
  auto center_path = *generateCenterLinePath(planner_data);
  auto rival_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                           planner_data->dynamic_object->objects, 2.8);

  bool prev_state_is_after_overtake = context_->getPreviousState().getName() == "AfterOvertake";

  auto transition_to_running_straight_line_state_based_on_prev_state = [&]() {
    if (!prev_state_is_after_overtake)
    {
      context_->transitionTo<ModuleNotLaunched>();
    }
    else
    {
      context_->transitionTo<AfterOvertake>(current_course_shift_length_);
    }
  };

  if (!rival_vehicle.has_value() ||
      rival_vehicle->longitudinal_from_ego < context_->getParameters().too_close_to_overtake_distance ||
      rival_vehicle->longitudinal_from_ego > context_->getParameters().prepare_overtake_distance)
  {
    transition_to_running_straight_line_state_based_on_prev_state();
    return;
  }

  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().start_overtake_distance)
  {
    context_->transitionTo<Overtaking>(current_course_shift_length_);
    return;
  }
}

std::string Approach::getName() const
{
  return "Approach";
}

void Approach::getPath(PlannerDataPtr planner_data, BehaviorModuleOutput* output)
{
  auto center_path = *generateCenterLinePath(planner_data);
  auto rival_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                           planner_data->dynamic_object->objects,
                                                           context_->getParameters().ego_course_width);
  auto [overtaking_path, overtaking_finish_pose, after_overtake_shift_length] =
      util::calcOvertakePath(output->path, rival_vehicle->object, current_course_shift_length_);
  output->path = overtaking_path;
  return;
}

void Overtaking::update(PlannerDataPtr planner_data)
{
  if (is_first_time_to_plan_)
    return;
  auto center_path = *generateCenterLinePath(planner_data);
  auto self_pose_frenet = convertToFrenetPoint(center_path.points, planner_data->self_odometry->pose.pose.position, 0);
  auto overtake_finish_pose_frenet = convertToFrenetPoint(center_path.points, overtake_finish_pose_.position, 0);
  if (self_pose_frenet.length > overtake_finish_pose_frenet.length)
  {
    context_->transitionTo<AfterOvertake>(after_overtake_shift_length_);
  }
}

std::string Overtaking::getName() const
{
  return "Overtaking";
}

void Overtaking::getPath(PlannerDataPtr planner_data, BehaviorModuleOutput* output)
{
  if (is_first_time_to_plan_)
  {
    auto center_path = *generateCenterLinePath(planner_data);
    auto rival_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                             planner_data->dynamic_object->objects,
                                                             context_->getParameters().ego_course_width);
    auto [overtaking_path, overtaking_finish_pose, after_overtake_shift_length] =
        util::calcOvertakePath(output->path, rival_vehicle->object, current_course_shift_length_);

    overtake_finish_pose_ = overtaking_finish_pose;
    overtaking_path_ = overtaking_path;
    after_overtake_shift_length_ = after_overtake_shift_length;
  }

  output->path = overtaking_path_;
  is_first_time_to_plan_ = false;
  return;
}

void AfterOvertake::update(PlannerDataPtr planner_data)
{
  auto center_path = *generateCenterLinePath(planner_data);
  auto front_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                           planner_data->dynamic_object->objects);
  if (!front_vehicle.has_value())
  {
    context_->transitionTo<BackToCenter>(after_overtake_shift_length_);
    return;
  }
  auto rival_vehicle = util::detectRivalVehicleInEgoCourse(planner_data->self_odometry->pose.pose, center_path,
                                                           planner_data->dynamic_object->objects,
                                                           context_->getParameters().ego_course_width);
  if (front_vehicle->longitudinal_from_ego < rival_vehicle->longitudinal_from_ego)
  {
    return;
  }
  if (!rival_vehicle.has_value())
  {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().too_close_to_overtake_distance)
  {
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().start_overtake_distance)
  {
    context_->transitionTo<Overtaking>(after_overtake_shift_length_);
    return;
  }
  if (rival_vehicle->longitudinal_from_ego < context_->getParameters().prepare_overtake_distance)
  {
    context_->transitionTo<Approach>(after_overtake_shift_length_);
    return;
  }
  return;
}

std::string AfterOvertake::getName() const
{
  return "AfterOvertake";
}

void AfterOvertake::getPath(PlannerDataPtr, BehaviorModuleOutput* output)
{
  util::addLateralOffset(&output->path, after_overtake_shift_length_);
}

void BackToCenter::update(PlannerDataPtr planner_data)
{
  if (is_first_time_to_plan_)
    return;
  auto center_path = *generateCenterLinePath(planner_data);
  auto self_pose_frenet = convertToFrenetPoint(center_path.points, planner_data->self_odometry->pose.pose.position, 0);
  auto back_to_center_finish_pose_frenet =
      convertToFrenetPoint(center_path.points, back_to_center_end_pose_.position, 0);
  if (self_pose_frenet.length > back_to_center_finish_pose_frenet.length)
  {
    context_->transitionTo<ModuleNotLaunched>();
  }
}

std::string BackToCenter::getName() const
{
  return "BackToCenter";
}

void BackToCenter::getPath(PlannerDataPtr planner_data, BehaviorModuleOutput* output)
{
  if (is_first_time_to_plan_)
  {
    auto center_path = *generateCenterLinePath(planner_data);
    auto [back_to_center_path, back_to_center_end_pose] = util::calcBackToCenterPath(
        center_path, planner_data->self_odometry->pose.pose, after_overtake_shift_length_,
        context_->getParameters().back_to_center_start_distance, context_->getParameters().back_to_center_end_distance);
    back_to_center_end_pose_ = back_to_center_end_pose;
    back_to_center_path_ = back_to_center_path;
  }
  output->path = back_to_center_path_;
  is_first_time_to_plan_ = false;
  return;
}

Context::Context(const RacingOvertakeParameters& parameters)
  : parameters_(parameters), state_(nullptr), previous_state_(nullptr)
{
  transitionTo<ModuleNotLaunched>();
}

void Context::updateState(PlannerDataPtr planner_data)
{
  state_->update(planner_data);
}

RacingOverTakeState& Context::getState() const
{
  return *state_;
}

RacingOverTakeState& Context::getPreviousState() const
{
  return *previous_state_;
}

const RacingOvertakeParameters& Context::getParameters() const
{
  return parameters_;
}

void Context::updateParameters(const RacingOvertakeParameters& parameters)
{
  parameters_ = parameters;
}

}  // namespace behavior_path_planner::racing_overtake::state