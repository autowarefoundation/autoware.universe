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

#ifndef MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H
#define MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H

#include <memory>
#include <map>
#include <string>
#include "utils_act/act_utils.hpp"
#include <variant>

namespace ns_states
{
enum class motionStateEnums : int
{
  isAtCompleteStop = 0, // there is no trajectory to follow
  isStoppedWillMove = 1, // vehicle is in stopped state will start moving
  willbeStopping = -1, // vehicle will be moving
  isMoving = 2, // vehicle is moving.
  isInEmergency = -2, // vehicle is in emergency state.
};

namespace state
{

/**
 * Individual states
 * */
struct isStoppedwillMove
{
  motionStateEnums state_type{motionStateEnums::isStoppedWillMove};
};

struct isatCompleteStop
{
  motionStateEnums state_type{motionStateEnums::isAtCompleteStop};
};

struct willbeStopping
{
  motionStateEnums state_type{motionStateEnums::willbeStopping};
};

struct isMoving
{
  motionStateEnums state_type{motionStateEnums::isMoving};
};

} // namespace state

using State = std::variant<state::isatCompleteStop, state::isMoving,
                           state::willbeStopping, state::isStoppedwillMove>;

template<class... Ts>
struct overload : Ts ...
{
  using Ts::operator()...;
};

/** creates overloaded  lambda functions state_type = []()(state_type as args ){}*/
template<class... Ts>
overload(Ts...) -> overload<Ts...>;

/** MAIN VARIANT FSM */
class VehicleMotionFSM
{
 public:
  VehicleMotionFSM() = default;

  VehicleMotionFSM(double const &stop_entry_ego_speed,
                   double const &stop_entry_target_speed,
                   double const &keep_stopping_distance,
                   double const &will_stop_distance);

  [[nodiscard]] State onEvent(state::isStoppedwillMove) const;

  [[nodiscard]] State onEvent(state::isMoving) const;

  [[nodiscard]] State onEvent(state::willbeStopping) const;

  [[nodiscard]] State onEvent(state::isatCompleteStop) const;

  void toggle(std::array<double, 3> const &dist2stop_egovx_nextvx);
  void printCurrentStateMsg();
  [[nodiscard]] motionStateEnums getCurrentStateType() const;

  std::string_view getFSMTypeReport();

 private:
  State current_state_{state::isStoppedwillMove{}};
  motionStateEnums current_state_type_{motionStateEnums::isStoppedWillMove};

  // State transition variables ; an array of [distance_to_stop, vcurrent, vnext]
  // Transition condition variables.
  double stop_state_entry_ego_speed_{0.2}; // [m/s]
  double stop_state_entry_target_speed_{0.5}; // [m]
  double stop_state_keep_stopping_dist_{0.5}; // [m]
  double will_stop_dist_{2.0}; // [m]

  std::array<double, 3> state_transition_vars_{};

  std::map<motionStateEnums, std::string> enum_texts{
    {motionStateEnums::isAtCompleteStop, "Vehicle is at complete stop."},
    {motionStateEnums::isStoppedWillMove, "Vehicle is stopping and will be moving."},
    {motionStateEnums::willbeStopping, "Vehicle will stop."},
    {motionStateEnums::isMoving, "Vehicle is moving."},
    {motionStateEnums::isInEmergency, "Vehicle is in EMERGENCY state."}};

};

} // namespace ns_states
#endif //MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H
