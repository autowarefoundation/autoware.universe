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

#include "state_machine.hpp"

// Gear shift checks
struct GearVelocity
{
  uint8_t start_gear;
  uint8_t target_gear;
  float velocity;
  uint8_t result_gear;
};

constexpr decltype(VO::velocity_mps) velocity_threshold{0.5F};

class GearShiftVelocity : public state_machine, public ::testing::WithParamInterface<GearVelocity>
{
protected:
  // Make sure the state report and command constants are the same because we're mixing stuff
  void SetUp()
  {
    ASSERT_FLOAT_EQ(velocity_threshold, config_.gear_shift_velocity_threshold());
    // Gear
    ASSERT_EQ(VSR::GEAR_DRIVE, VSC::GEAR_DRIVE);
    ASSERT_EQ(VSR::GEAR_PARK, VSC::GEAR_PARK);
    ASSERT_EQ(VSR::GEAR_REVERSE, VSC::GEAR_REVERSE);
  }
};

TEST_P(GearShiftVelocity, Basic)
{
  const auto param = GetParam();
  // Set up state
  {
    const auto prior_state = VSR{}.set__gear(param.start_gear);
    const auto prior_odom = VO{}.set__velocity_mps(param.velocity);
    sm_.update(prior_odom, prior_state);
  }
  // Try a command
  const auto state = VSC{}.set__gear(param.target_gear);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  EXPECT_EQ(cmd.control(), ctrl);
  // All equal except for gear
  EXPECT_EQ(cmd.state()->wiper, state.wiper);
  EXPECT_EQ(cmd.state()->mode, state.mode);
  EXPECT_EQ(cmd.state()->blinker, state.blinker);
  EXPECT_EQ(cmd.state()->headlight, state.headlight);
  EXPECT_EQ(cmd.state()->hand_brake, state.hand_brake);
  EXPECT_EQ(cmd.state()->horn, state.horn);
  // gear
  EXPECT_EQ(cmd.state()->gear, param.result_gear);
  // report
  if (param.result_gear != param.target_gear) {
    EXPECT_TRUE(has_report(StateMachineReport::REMOVE_GEAR_COMMAND));
  } else {
    EXPECT_TRUE(sm_.reports().empty());
  }
}

INSTANTIATE_TEST_CASE_P(
  Test,
  GearShiftVelocity,
  ::testing::Values(
    // Drive to park
    GearVelocity{VSR::GEAR_DRIVE, VSC::GEAR_PARK, velocity_threshold + 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_DRIVE, VSC::GEAR_PARK, velocity_threshold, VSC::GEAR_PARK},
    // Drive to reverse
    GearVelocity{
  VSR::GEAR_DRIVE, VSC::GEAR_REVERSE, velocity_threshold + 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{
  VSR::GEAR_DRIVE, VSC::GEAR_REVERSE, velocity_threshold, VSC::GEAR_REVERSE},
    // Low to reverse
    GearVelocity{VSR::GEAR_LOW, VSC::GEAR_REVERSE, velocity_threshold + 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_LOW, VSC::GEAR_REVERSE, velocity_threshold, VSC::GEAR_REVERSE},
    // Low to park
    GearVelocity{VSR::GEAR_LOW, VSC::GEAR_PARK, velocity_threshold + 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_LOW, VSC::GEAR_PARK, velocity_threshold, VSC::GEAR_PARK},
    // Reverse to park
    GearVelocity{
  VSR::GEAR_REVERSE, VSC::GEAR_PARK, -velocity_threshold - 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_REVERSE, VSC::GEAR_PARK, velocity_threshold, VSC::GEAR_PARK},
    // Reverse to drive
    GearVelocity{
  VSR::GEAR_REVERSE, VSC::GEAR_DRIVE, -velocity_threshold - 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_REVERSE, VSC::GEAR_DRIVE, velocity_threshold, VSC::GEAR_DRIVE},
    // Reverse to low
    GearVelocity{
  VSR::GEAR_REVERSE, VSC::GEAR_LOW, -velocity_threshold - 1.0F, VSC::GEAR_NO_COMMAND},
    GearVelocity{VSR::GEAR_REVERSE, VSC::GEAR_LOW, velocity_threshold, VSC::GEAR_LOW}
    // cppcheck-suppress syntaxError
  ),
);

////////////////////////////////////////////////////////////////////////////////
// reverse <--> drive logic
struct AutoGear
{
  float velocity;
  float accel;
  uint8_t prior_gear;
  uint8_t gear_command;
  uint8_t expected_gear;
};

class AutoGearShift : public state_machine, public ::testing::WithParamInterface<AutoGear>
{
};

TEST_P(AutoGearShift, Basic)
{
  ASSERT_EQ(config_.time_step(), std::chrono::milliseconds{100LL});
  ASSERT_GT(config_.auto_gear_shift_accel_deadzone(), 0.1F);
  const auto param = GetParam();
  // Set up state
  {
    const auto prior_state = VSR{}.set__gear(param.prior_gear);
    const auto prior_odom = VO{}.set__velocity_mps(param.velocity);
    sm_.update(prior_odom, prior_state);
  }
  // Try a command
  const auto state = VSC{}.set__gear(param.gear_command);
  const auto control = VCC{}.set__long_accel_mps2(param.accel);
  const auto cmd = sm_.compute_safe_commands(Command{control, state});
  EXPECT_EQ(cmd.control(), control);
  // All equal except for gear
  EXPECT_EQ(cmd.state()->wiper, state.wiper);
  EXPECT_EQ(cmd.state()->mode, state.mode);
  EXPECT_EQ(cmd.state()->blinker, state.blinker);
  EXPECT_EQ(cmd.state()->headlight, state.headlight);
  EXPECT_EQ(cmd.state()->hand_brake, state.hand_brake);
  EXPECT_EQ(cmd.state()->horn, state.horn);
  // gear
  EXPECT_EQ(cmd.state()->gear, param.expected_gear);
  // no report
  EXPECT_TRUE(sm_.reports().empty());
}

// Assume characteristic time step of 100ms
INSTANTIATE_TEST_CASE_P(
  Test,
  AutoGearShift,
  ::testing::Values(
    // Normal case: forward driving, do nothing: 0-5
    AutoGear{10.0F, 3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{1.0F, 3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.0F, 3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{10.0F, 0.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{1.0F, 0.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.0F, 0.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.0F, -0.1F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    // Normal case: reverse driving, do nothing: 6-11
    AutoGear{-10.0F, -3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-1.0F, -3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.0F, -3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-10.0F, 0.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-1.0F, 0.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-0.0F, 0.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-0.0F, 0.1F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    // Forward driving deceleration: 12-15
    AutoGear{10.0F, -3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.3F, -3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{0.2F, -3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_REVERSE},
    AutoGear{0.0F, -3.0F, VSR::GEAR_DRIVE, VSC::GEAR_NO_COMMAND, VSC::GEAR_REVERSE},
    // Backward driving deceleration: 16-19
    AutoGear{-10.0F, 3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-0.3F, 3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_NO_COMMAND},
    AutoGear{-0.2F, 3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_DRIVE},
    AutoGear{-0.0F, 3.0F, VSR::GEAR_REVERSE, VSC::GEAR_NO_COMMAND, VSC::GEAR_DRIVE}
    // TODO(c.ho) bad/inconsistent states
  ),
);
