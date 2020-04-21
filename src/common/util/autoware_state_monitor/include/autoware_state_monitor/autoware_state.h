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
  if (state == autoware_system_msgs::AutowareState::Error) return AutowareState::Error;
  if (state == autoware_system_msgs::AutowareState::InitializingVehicle)
    return AutowareState::InitializingVehicle;
  if (state == autoware_system_msgs::AutowareState::WaitingForRoute)
    return AutowareState::WaitingForRoute;
  if (state == autoware_system_msgs::AutowareState::Planning) return AutowareState::Planning;
  if (state == autoware_system_msgs::AutowareState::WaitingForEngage)
    return AutowareState::WaitingForEngage;
  if (state == autoware_system_msgs::AutowareState::Driving) return AutowareState::Driving;
  if (state == autoware_system_msgs::AutowareState::ArrivedGoal) return AutowareState::ArrivedGoal;
  if (state == autoware_system_msgs::AutowareState::FailedToArriveGoal)
    return AutowareState::FailedToArriveGoal;
  return AutowareState::Error;
}

inline std::string toString(const AutowareState & state)
{
  if (state == AutowareState::Error) return autoware_system_msgs::AutowareState::Error;
  if (state == AutowareState::InitializingVehicle)
    return autoware_system_msgs::AutowareState::InitializingVehicle;
  if (state == AutowareState::WaitingForRoute)
    return autoware_system_msgs::AutowareState::WaitingForRoute;
  if (state == AutowareState::Planning) return autoware_system_msgs::AutowareState::Planning;
  if (state == AutowareState::WaitingForEngage)
    return autoware_system_msgs::AutowareState::WaitingForEngage;
  if (state == AutowareState::Driving) return autoware_system_msgs::AutowareState::Driving;
  if (state == AutowareState::ArrivedGoal) return autoware_system_msgs::AutowareState::ArrivedGoal;
  if (state == AutowareState::FailedToArriveGoal)
    return autoware_system_msgs::AutowareState::FailedToArriveGoal;
  return autoware_system_msgs::AutowareState::Error;
}
