// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

#include <common/types.hpp>
#include <gtest/gtest.h>

#include <vehicle_interface/safety_state_machine.hpp>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

using autoware::drivers::vehicle_interface::Command;
using autoware::drivers::vehicle_interface::Limits;
using autoware::drivers::vehicle_interface::SafetyStateMachine;
using autoware::drivers::vehicle_interface::StateMachineConfig;
using autoware::drivers::vehicle_interface::StateMachineReport;

using autoware_auto_vehicle_msgs::msg::WipersCommand;
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using VO = autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using VSR = autoware_auto_vehicle_msgs::msg::VehicleStateReport;
using VSC = autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
using VCC = autoware_auto_vehicle_msgs::msg::VehicleControlCommand;

class state_machine : public ::testing::Test
{
protected:
  bool8_t has_report(const StateMachineReport rpt)
  {
    const auto & reports = sm_.reports();
    const auto res = std::find(reports.cbegin(), reports.cend(), rpt);
    return res != reports.cend();
  }

  StateMachineConfig config_{
    0.5F,  // gear_shift_velocity_threshold
    Limits<float32_t>{-3.0F, 3.0F, 1.0F},  // accel limits
    Limits<float32_t>{-0.331F, 0.331F, 0.3F},  // front steer limits
    std::chrono::milliseconds{100LL},  // time_step
    3.0F,  // timeout acceleration
    std::chrono::seconds{3LL},  // state transition timeout
    0.5F  // gear shift accel deadzone
  };
  SafetyStateMachine sm_{config_};
  const VCC ctrl{};  // default ctrl
};  // class state_machine

#endif  // STATE_MACHINE_HPP_
