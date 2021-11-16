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

struct WiperHeadlight
{
  uint8_t wiper;
  uint8_t headlight;
  uint8_t headlight_result;
};

class wiper_headlight_state_machine
  : public state_machine, public ::testing::WithParamInterface<WiperHeadlight>
{
protected:
  // Make sure the state report and command constants are the same because we're mixing stuff
  void SetUp()
  {
    // Wiper
    ASSERT_EQ(VSR::WIPER_LOW, WipersCommand::ENABLE_LOW);
    ASSERT_EQ(VSR::WIPER_HIGH, WipersCommand::ENABLE_HIGH);
    ASSERT_EQ(VSR::WIPER_OFF, WipersCommand::DISABLE);
    // headlight
    ASSERT_EQ(VSR::HEADLIGHT_OFF, HeadlightsCommand::DISABLE);
    ASSERT_EQ(VSR::HEADLIGHT_ON, HeadlightsCommand::ENABLE_LOW);
    ASSERT_EQ(VSR::HEADLIGHT_HIGH, HeadlightsCommand::ENABLE_HIGH);
  }
};

class WipersOnHeadlightsOn : public wiper_headlight_state_machine
{
};

// Turning wipers on should turn on headlights: wiper command, light command, light result
TEST_P(WipersOnHeadlightsOn, Basic)
{
  const auto param = GetParam();
  const auto state = VSC{}.set__wiper(param.wiper).set__headlight(param.headlight);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  EXPECT_EQ(cmd.control(), ctrl);
  // All equal except for lamp
  EXPECT_EQ(cmd.state()->wiper, state.wiper);
  EXPECT_EQ(cmd.state()->mode, state.mode);
  EXPECT_EQ(cmd.state()->blinker, state.blinker);
  EXPECT_EQ(cmd.state()->gear, state.gear);
  EXPECT_EQ(cmd.state()->hand_brake, state.hand_brake);
  EXPECT_EQ(cmd.state()->horn, state.horn);
  // Lamp
  EXPECT_EQ(cmd.state()->headlight, param.headlight_result);
  // report only if changed
  if (param.headlight_result != param.headlight) {
    EXPECT_TRUE(has_report(StateMachineReport::WIPERS_ON_HEADLIGHTS_ON));
  }
}

INSTANTIATE_TEST_CASE_P(
  Test,
  WipersOnHeadlightsOn,
  ::testing::Values(
    WiperHeadlight{WipersCommand::ENABLE_LOW, HeadlightsCommand::NO_COMMAND,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_LOW, HeadlightsCommand::DISABLE,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_LOW, HeadlightsCommand::ENABLE_LOW,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_LOW, HeadlightsCommand::ENABLE_HIGH,
      HeadlightsCommand::ENABLE_HIGH},
    WiperHeadlight{WipersCommand::ENABLE_HIGH, HeadlightsCommand::NO_COMMAND,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_HIGH, HeadlightsCommand::DISABLE,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_HIGH, HeadlightsCommand::ENABLE_LOW,
      HeadlightsCommand::ENABLE_LOW},
    WiperHeadlight{WipersCommand::ENABLE_HIGH, HeadlightsCommand::ENABLE_HIGH,
      HeadlightsCommand::ENABLE_HIGH}
    // cppcheck-suppress syntaxError
  ),
);

class WipersOffHeadlightNoChange : public wiper_headlight_state_machine
{
};

// Turning off wipers should keep headlights on: wiper state, light state, light command
TEST_P(WipersOffHeadlightNoChange, Basic)
{
  const auto param = GetParam();
  // Set state to wipers on, headlights on
  sm_.update(VO{}, VSR{}.set__headlight(param.headlight).set__wiper(param.wiper));
  // Turn off wipers
  const auto state =
    VSC{}.set__wiper(WipersCommand::DISABLE).set__headlight(param.headlight_result);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  // Nothing should change
  EXPECT_EQ(cmd.control(), ctrl);
  EXPECT_EQ(cmd.state(), state);
  // report
  EXPECT_TRUE(sm_.reports().empty());
}

INSTANTIATE_TEST_CASE_P(
  Test,
  WipersOffHeadlightNoChange,
  ::testing::Values(
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, HeadlightsCommand::NO_COMMAND},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_ON, HeadlightsCommand::ENABLE_HIGH},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, HeadlightsCommand::NO_COMMAND},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_LOW, VSR::HEADLIGHT_HIGH, HeadlightsCommand::ENABLE_HIGH},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, HeadlightsCommand::NO_COMMAND},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_ON, HeadlightsCommand::ENABLE_HIGH},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, HeadlightsCommand::NO_COMMAND},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, HeadlightsCommand::DISABLE},
    WiperHeadlight{VSR::WIPER_HIGH, VSR::HEADLIGHT_HIGH, HeadlightsCommand::ENABLE_HIGH}
  ),
);
