/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
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


#include <cmath>
#include "nonlinear_mpc_node/nonlinear_mpc_state_machine.hpp"

namespace ns_states
{

VehicleMotionFSM::VehicleMotionFSM(const double &stop_entry_ego_speed,
                                   const double &stop_entry_target_speed,
                                   const double &keep_stopping_distance,
                                   const double &will_stop_distance)
  : stop_state_entry_ego_speed_{stop_entry_ego_speed},
    stop_state_entry_target_speed_{stop_entry_target_speed},
    stop_state_keep_stopping_dist_{keep_stopping_distance},
    will_stop_dist_{will_stop_distance}
{}

void VehicleMotionFSM::toggle(const std::array<double, 3> &dist2stop_egovx_nextvx)
{
  state_transition_vars_ = dist2stop_egovx_nextvx;

  current_state_ = std::visit(overload
                                {
                                  [this](auto const &current_state)
                                  {
                                    return onEvent(current_state);
                                  }
                                }, current_state_);

  current_state_type_ = std::visit(overload
                                     {
                                       [](auto const &current_state)
                                       { return current_state.state_type; }
                                     }, current_state_);
}

void VehicleMotionFSM::printCurrentStateMsg()
{
  ns_utils::print(" State event: ", enum_texts[current_state_type_]);
}

motionStateEnums VehicleMotionFSM::getCurrentStateType() const
{
  return current_state_type_;
}

State VehicleMotionFSM::onEvent(state::isStoppedwillMove) const
{
  auto const &dist_to_stop_point = state_transition_vars_[0];
  auto const &ego_speed = std::fabs(state_transition_vars_[1]);

  if (dist_to_stop_point > stop_state_keep_stopping_dist_ &&
      ego_speed < stop_state_entry_ego_speed_)
  {
    return state::isStoppedwillMove{};
  }

  if (dist_to_stop_point > stop_state_keep_stopping_dist_ &&
      ego_speed > stop_state_entry_ego_speed_)
  {
    return state::isMoving{};
  }

  return state::isStoppedwillMove{};
}
State VehicleMotionFSM::onEvent(state::isMoving) const
{
  auto const &dist_to_stop_point = state_transition_vars_[0];
  auto const &ego_speed = std::fabs(state_transition_vars_[1]);

  if (dist_to_stop_point < will_stop_dist_)
  {
    return state::willbeStopping{};
  }

  if (dist_to_stop_point < stop_state_keep_stopping_dist_ &&
      ego_speed < stop_state_entry_target_speed_)
  {
    return state::isatCompleteStop{};
  }

  return state::isMoving{};
}
State VehicleMotionFSM::onEvent(state::willbeStopping) const
{
  auto const &dist_to_stop_point = state_transition_vars_[0];
  auto const &ego_speed = std::fabs(state_transition_vars_[1]);

  if (auto const &next_point_speed_ref = std::fabs(state_transition_vars_[2]);
    dist_to_stop_point < will_stop_dist_ && ego_speed < stop_state_entry_ego_speed_
    && next_point_speed_ref < stop_state_entry_target_speed_)
  {
    return state::isatCompleteStop{};
  }

  return state::willbeStopping{};
}

State VehicleMotionFSM::onEvent(state::isatCompleteStop) const
{
  auto const &dist_to_stop_point = state_transition_vars_[0];
  auto const &ego_speed = std::fabs(state_transition_vars_[1]);
  // auto const &next_point_speed_ref = std::fabs(state_transition_vars_[2]);

  if (dist_to_stop_point > stop_state_keep_stopping_dist_ &&
      ego_speed <= stop_state_entry_ego_speed_)
  {
    return state::isStoppedwillMove{};
  }

  if (dist_to_stop_point > stop_state_keep_stopping_dist_ &&
      ego_speed > stop_state_entry_ego_speed_)
  {
    return state::isMoving{};
  }

  return state::isatCompleteStop{};
}

std::string_view VehicleMotionFSM::getFSMTypeReport()
{
  return enum_texts[current_state_type_];
}
}  // namespace ns_states
