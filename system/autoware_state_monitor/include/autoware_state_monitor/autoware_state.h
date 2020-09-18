/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#pragma once

#include <string>

#include <autoware_system_msgs/AutowareState.h>

enum class AutowareState : int8_t {
  Error = -1,
  InitializingVehicle = 0,
  WaitingForRoute,
  Planning,
  WaitingForEngage,
  Driving,
  ArrivedGoal,
  FailedToArriveGoal,
};

inline AutowareState fromString(const std::string & state)
{
  using StateMessage = autoware_system_msgs::AutowareState;

  if (state == StateMessage::Error) return AutowareState::Error;
  if (state == StateMessage::InitializingVehicle) return AutowareState::InitializingVehicle;
  if (state == StateMessage::WaitingForRoute) return AutowareState::WaitingForRoute;
  if (state == StateMessage::Planning) return AutowareState::Planning;
  if (state == StateMessage::WaitingForEngage) return AutowareState::WaitingForEngage;
  if (state == StateMessage::Driving) return AutowareState::Driving;
  if (state == StateMessage::ArrivedGoal) return AutowareState::ArrivedGoal;
  if (state == StateMessage::FailedToArriveGoal) return AutowareState::FailedToArriveGoal;

  return AutowareState::Error;
}

inline std::string toString(const AutowareState & state)
{
  using StateMessage = autoware_system_msgs::AutowareState;

  if (state == AutowareState::Error) return StateMessage::Error;
  if (state == AutowareState::InitializingVehicle) return StateMessage::InitializingVehicle;
  if (state == AutowareState::WaitingForRoute) return StateMessage::WaitingForRoute;
  if (state == AutowareState::Planning) return StateMessage::Planning;
  if (state == AutowareState::WaitingForEngage) return StateMessage::WaitingForEngage;
  if (state == AutowareState::Driving) return StateMessage::Driving;
  if (state == AutowareState::ArrivedGoal) return StateMessage::ArrivedGoal;
  if (state == AutowareState::FailedToArriveGoal) return StateMessage::FailedToArriveGoal;

  return StateMessage::Error;
}
