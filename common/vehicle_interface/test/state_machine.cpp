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

#include <common/types.hpp>

#include <time_utils/time_utils.hpp>

#include <chrono>
#include <utility>
#include <vector>

#include "state_machine.hpp"

using autoware::common::types::float32_t;

////////////////////////////////////////////////////////////////////////////////
enum class FrequencyTarget
{
  SteerReport,
  VelocityReport,
  AccelCommand,
  SteerCommand
};

using FrequencyTargets = std::vector<FrequencyTarget>;

struct HighFrequency
{
  float32_t bias;
  float32_t amplitude;
  std::chrono::nanoseconds period;
  std::size_t count;
  FrequencyTargets targets;
};

class HighFrequencyWarning
  : public state_machine, public ::testing::WithParamInterface<HighFrequency>
{
protected:
  using Time = builtin_interfaces::msg::Time;
  using Signal = std::vector<std::pair<float32_t, Time>>;
  Signal make_signal(const HighFrequency & param)
  {
    Signal ret{param.count};
    const auto start = std::chrono::system_clock::now();
    for (auto idx = 0U; idx < param.count; ++idx) {
      const auto offset = idx % 2U == 0U ? param.amplitude : -param.amplitude;
      const auto val = offset + param.amplitude;
      const auto time = ::time_utils::to_message(start + ((param.period * idx) / 2U));
      ret[idx] = {val, time};
    }
    return ret;
  }
  void set_state(const Signal & signal, const FrequencyTargets & targets)
  {
    const auto do_steer =
      std::find(targets.begin(), targets.end(), FrequencyTarget::SteerReport) == targets.begin();
    const auto do_velocity =
      std::find(targets.begin(), targets.end(), FrequencyTarget::VelocityReport) == targets.begin();
    if ((!do_steer) && (!do_velocity)) {
      return;
    }
    for (const auto & val : signal) {
      auto odom = VO{}.set__stamp(val.second);
      if (do_velocity) {
        odom.set__velocity_mps(val.first);
      }
      if (do_steer) {
        odom.set__front_wheel_angle_rad(val.first);
      }
      const auto state = VSR{}.set__stamp(val.second);
      sm_.update(odom, state);
    }
  }
  void compute_command(const Signal & signal, const FrequencyTargets & targets)
  {
    const auto do_steer =
      std::find(targets.begin(), targets.end(), FrequencyTarget::SteerCommand) == targets.begin();
    const auto do_accel =
      std::find(targets.begin(), targets.end(), FrequencyTarget::AccelCommand) == targets.begin();
    if ((!do_steer) && (!do_accel)) {
      return;
    }
    for (const auto & val : signal) {
      auto ctrl = VCC{}.set__stamp(val.second);
      if (do_accel) {
        ctrl.set__long_accel_mps2(val.first);
      }
      if (do_steer) {
        ctrl.set__front_wheel_angle_rad(val.first);
      }
      (void)sm_.compute_safe_commands(Command{ctrl});
    }
  }
};

TEST_P(HighFrequencyWarning, Basic)
{
  GTEST_SKIP();  // #5455: Not quarantined since not broken... just not implemented
  const auto param = GetParam();
  const auto signal = make_signal(param);
  // Shove high frequency reports in
  set_state(signal, param.targets);
  // Check result
  for (const auto target : param.targets) {
    switch (target) {
      case FrequencyTarget::SteerReport:
        EXPECT_TRUE(has_report(StateMachineReport::HIGH_FREQUENCY_STEER_REPORT));
        break;
      case FrequencyTarget::VelocityReport:
        EXPECT_TRUE(has_report(StateMachineReport::HIGH_FREQUENCY_VELOCITY_REPORT));
        break;
      case FrequencyTarget::AccelCommand:
      case FrequencyTarget::SteerCommand:
        EXPECT_TRUE(sm_.reports().empty());
    }
  }
  // Shove high frequency commands in
  compute_command(signal, param.targets);
  // Check result
  for (const auto target : param.targets) {
    switch (target) {
      case FrequencyTarget::AccelCommand:
        EXPECT_TRUE(has_report(StateMachineReport::HIGH_FREQUENCY_ACCELERATION_COMMAND));
        break;
      case FrequencyTarget::SteerCommand:
        EXPECT_TRUE(has_report(StateMachineReport::HIGH_FREQUENCY_STEER_COMMAND));
        break;
      case FrequencyTarget::VelocityReport:
      case FrequencyTarget::SteerReport:
        EXPECT_TRUE(sm_.reports().empty());
    }
  }
}

/* TODO(Maxime CLEMENT): migrating from INSTANTIATE_TEST_CASE_P (deprecated) causes errors
INSTANTIATE_TEST_SUITE_P(
  Test,
  HighFrequencyWarning,
  ::testing::Values(
    HighFrequency{0.0F, 0.3F, std::chrono::milliseconds{10LL}, 20U,
      FrequencyTargets{FrequencyTarget::SteerReport}},
    HighFrequency{10.0F, 10.0F, std::chrono::milliseconds{10LL}, 20U,
      FrequencyTargets{FrequencyTarget::VelocityReport}},
    HighFrequency{0.0F, 2.0F, std::chrono::milliseconds{10LL}, 20U,
      FrequencyTargets{FrequencyTarget::AccelCommand}},
    HighFrequency{0.0F, 0.3F, std::chrono::milliseconds{10LL}, 20U,
      FrequencyTargets{FrequencyTarget::SteerCommand}}
    // cppcheck-suppress syntaxError
  ),
);
*/

////////////////////////////////////////////////////////////////////////////////
struct StateChange
{
  VSR start;
  VSC command;
};

class NoStateChange : public state_machine, public ::testing::WithParamInterface<StateChange>
{
};

TEST_P(NoStateChange, Basic)
{
  const auto param = GetParam();
  const auto now = std::chrono::system_clock::now();
  // Set initial state
  {
    const auto now1 = ::time_utils::to_message(now);
    const auto start = VSR{param.start}.set__stamp(now1);
    sm_.update(VO{}.set__stamp(now1), start);
  }
  // Apply command
  {
    constexpr auto dt = std::chrono::milliseconds{1LL};
    ASSERT_LT(dt, config_.state_transition_timeout());
    const auto now2 = ::time_utils::to_message(now + dt);
    const auto ctrl2 = VCC{ctrl}.set__stamp(now2);
    const auto state = VSC{param.command}.set__stamp(now2);
    const auto cmd = sm_.compute_safe_commands(Command{ctrl2, state});
    // Runs into issues with headlightxs-wiper logic: check not strictly needed
    // EXPECT_EQ(cmd.state().value(), state);
    EXPECT_EQ(ctrl2, cmd.control());
  }
  // Set terminal state
  {
    constexpr auto timeout_dt = std::chrono::seconds{10LL};
    ASSERT_GT(timeout_dt, config_.state_transition_timeout());
    const auto now3 = ::time_utils::to_message(now + timeout_dt);
    const auto end = VSR{param.start}.set__stamp(now3);
    sm_.update(VO{}.set__stamp(now3), end);
    EXPECT_TRUE(has_report(StateMachineReport::STATE_TRANSITION_TIMEOUT));
  }
}

/* TODO(Maxime CLEMENT): migrating from INSTANTIATE_TEST_CASE_P (deprecated) causes errors
INSTANTIATE_TEST_SUITE_P(
  Test,
  NoStateChange,
  ::testing::Values(
    //// Gear //// 0-15
    // Gear park->*
    StateChange{VSR{}.set__gear(VSR::GEAR_PARK), VSC{}.set__gear(VSC::GEAR_DRIVE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_PARK), VSC{}.set__gear(VSC::GEAR_NEUTRAL)},
    StateChange{VSR{}.set__gear(VSR::GEAR_PARK), VSC{}.set__gear(VSC::GEAR_REVERSE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_PARK), VSC{}.set__gear(VSC::GEAR_LOW)},
    // Gear drive->*
    StateChange{VSR{}.set__gear(VSR::GEAR_DRIVE), VSC{}.set__gear(VSC::GEAR_PARK)},
    StateChange{VSR{}.set__gear(VSR::GEAR_DRIVE), VSC{}.set__gear(VSC::GEAR_NEUTRAL)},
    StateChange{VSR{}.set__gear(VSR::GEAR_DRIVE), VSC{}.set__gear(VSC::GEAR_REVERSE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_DRIVE), VSC{}.set__gear(VSC::GEAR_LOW)},
    // Gear neutral->*
    StateChange{VSR{}.set__gear(VSR::GEAR_NEUTRAL), VSC{}.set__gear(VSC::GEAR_PARK)},
    StateChange{VSR{}.set__gear(VSR::GEAR_NEUTRAL), VSC{}.set__gear(VSC::GEAR_DRIVE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_NEUTRAL), VSC{}.set__gear(VSC::GEAR_REVERSE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_NEUTRAL), VSC{}.set__gear(VSC::GEAR_LOW)},
    // Gear low->*
    StateChange{VSR{}.set__gear(VSR::GEAR_LOW), VSC{}.set__gear(VSC::GEAR_PARK)},
    StateChange{VSR{}.set__gear(VSR::GEAR_LOW), VSC{}.set__gear(VSC::GEAR_DRIVE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_LOW), VSC{}.set__gear(VSC::GEAR_REVERSE)},
    StateChange{VSR{}.set__gear(VSR::GEAR_LOW), VSC{}.set__gear(VSC::GEAR_NEUTRAL)},
    //// Headlight //// 16-21
    // off -> *
    StateChange{VSR{}.set__headlight(VSR::HEADLIGHT_OFF),
      VSC{}.set__headlight(HeadlightsCommand::ENABLE_LOW)},
    StateChange{
  VSR{}.set__headlight(VSR::HEADLIGHT_OFF), VSC{}.set__headlight(HeadlightsCommand::ENABLE_HIGH)},
    // on -> *
    StateChange{VSR{}.set__headlight(VSR::HEADLIGHT_ON),
      VSC{}.set__headlight(HeadlightsCommand::DISABLE)},
    StateChange{VSR{}.set__headlight(VSR::HEADLIGHT_ON),
      VSC{}.set__headlight(HeadlightsCommand::ENABLE_HIGH)},
    // high -> *
    StateChange{VSR{}.set__headlight(VSR::HEADLIGHT_HIGH),
      VSC{}.set__headlight(HeadlightsCommand::ENABLE_LOW)},
    StateChange{
  VSR{}.set__headlight(VSR::HEADLIGHT_HIGH), VSC{}.set__headlight(HeadlightsCommand::DISABLE)},
    //// Blinker //// 22-33
    // off -> *
    StateChange{VSR{}.set__blinker(VSR::BLINKER_OFF), VSC{}.set__blinker(VSC::BLINKER_LEFT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_OFF), VSC{}.set__blinker(VSC::BLINKER_RIGHT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_OFF), VSC{}.set__blinker(VSC::BLINKER_HAZARD)},
    // left -> *
    StateChange{VSR{}.set__blinker(VSR::BLINKER_LEFT), VSC{}.set__blinker(VSC::BLINKER_OFF)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_LEFT), VSC{}.set__blinker(VSC::BLINKER_RIGHT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_LEFT), VSC{}.set__blinker(VSC::BLINKER_HAZARD)},
    // right -> *
    StateChange{VSR{}.set__blinker(VSR::BLINKER_RIGHT), VSC{}.set__blinker(VSC::BLINKER_LEFT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_RIGHT), VSC{}.set__blinker(VSC::BLINKER_OFF)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_RIGHT), VSC{}.set__blinker(VSC::BLINKER_HAZARD)},
    // hazard -> *
    StateChange{VSR{}.set__blinker(VSR::BLINKER_HAZARD), VSC{}.set__blinker(VSC::BLINKER_LEFT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_HAZARD), VSC{}.set__blinker(VSC::BLINKER_RIGHT)},
    StateChange{VSR{}.set__blinker(VSR::BLINKER_HAZARD), VSC{}.set__blinker(VSC::BLINKER_OFF)},
    //// Wiper //// 34-45
    // off -> *
    StateChange{VSR{}.set__wiper(VSR::WIPER_OFF), VSC{}.set__wiper(WipersCommand::ENABLE_LOW)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_OFF), VSC{}.set__wiper(WipersCommand::ENABLE_HIGH)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_OFF), VSC{}.set__wiper(WipersCommand::ENABLE_CLEAN)},
    // low -> *
    StateChange{VSR{}.set__wiper(VSR::WIPER_LOW), VSC{}.set__wiper(WipersCommand::DISABLE)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_LOW), VSC{}.set__wiper(WipersCommand::ENABLE_HIGH)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_LOW), VSC{}.set__wiper(WipersCommand::ENABLE_CLEAN)},
    // high -> *
    StateChange{VSR{}.set__wiper(VSR::WIPER_HIGH), VSC{}.set__wiper(WipersCommand::ENABLE_LOW)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_HIGH), VSC{}.set__wiper(WipersCommand::DISABLE)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_HIGH), VSC{}.set__wiper(WipersCommand::ENABLE_CLEAN)},
    // clean -> *
    StateChange{VSR{}.set__wiper(VSR::WIPER_CLEAN), VSC{}.set__wiper(WipersCommand::ENABLE_LOW)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_CLEAN), VSC{}.set__wiper(WipersCommand::ENABLE_HIGH)},
    StateChange{VSR{}.set__wiper(VSR::WIPER_CLEAN), VSC{}.set__wiper(WipersCommand::DISABLE)},
    // Everything 46
    StateChange{
  VSR{}.set__wiper(VSR::WIPER_CLEAN).set__blinker(VSR::BLINKER_HAZARD).
  set__headlight(VSR::HEADLIGHT_HIGH).set__gear(VSR::GEAR_DRIVE),
  VSC{}.set__wiper(WipersCommand::DISABLE).set__blinker(VSC::BLINKER_OFF).
  set__headlight(HeadlightsCommand::DISABLE).set__gear(VSC::GEAR_LOW)
}
    // TODO(c.ho) more combinatorial tests
  ),
);
*/

// TODO(c.ho) test cases for overriding a previous requested change

////////////////////////////////////////////////////////////////////////////////
struct TimeoutCommand
{
  decltype(VO::velocity_mps) velocity;
  decltype(VCC::long_accel_mps2) expected_accel;
};

class TimeoutCommands : public state_machine, public ::testing::WithParamInterface<TimeoutCommand>
{
protected:
  void SetUp()
  {
    ASSERT_FLOAT_EQ(config_.timeout_acceleration(), 3.0F);
    ASSERT_EQ(config_.time_step(), std::chrono::milliseconds{100LL});
  }
};

TEST_P(TimeoutCommands, Basic)
{
  const auto param = GetParam();
  // set state
  const auto odom = VO{}.set__velocity_mps(param.velocity);
  sm_.update(odom, VSR{});
  // Compute timeout
  const auto cmd = sm_.timeout_commands();
  EXPECT_EQ(cmd.state()->blinker, VSC::BLINKER_HAZARD);
  EXPECT_FLOAT_EQ(cmd.control().long_accel_mps2, param.expected_accel);
  EXPECT_FLOAT_EQ(cmd.control().front_wheel_angle_rad, 0.0F);
  EXPECT_FLOAT_EQ(cmd.control().rear_wheel_angle_rad, 0.0F);
}

// Assume characteristic time rate of 100ms

/* TODO(Maxime CLEMENT): migrating from INSTANTIATE_TEST_CASE_P (deprecated) causes errors
INSTANTIATE_TEST_SUITE_P(
  Test,
  TimeoutCommands,
  ::testing::Values(
    TimeoutCommand{0.0F, 0.0F},  // Stopped
    // Positive
    TimeoutCommand{10.0F, -3.0F},  // saturated case
    TimeoutCommand{0.1F, -1.0F},  // Near stop case
    // Negative
    TimeoutCommand{-10.0F, 3.0F},  // saturated case
    TimeoutCommand{-0.1F, 1.0F}  // Near stop case
  ),
);
*/
