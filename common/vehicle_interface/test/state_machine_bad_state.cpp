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

#include <vector>

#include "state_machine.hpp"

using autoware::common::types::bool8_t;

enum class State
{
  WIPER,
  BLINKER,
  GEAR,
  HEADLIGHT,
  MODE
};

struct BadStateCommandParam
{
  std::vector<State> bad_states;
  decltype(WipersCommand::ENABLE_CLEAN) bad_value;
};

class BadStateCommand
  : public state_machine, public ::testing::WithParamInterface<BadStateCommandParam>
{
protected:
  VSC make_bad_state(const BadStateCommandParam & param)
  {
    VSC ret{};
    const auto bad_val = param.bad_value;
    for (const auto state : param.bad_states) {
      switch (state) {
        case State::WIPER:
          ret.wiper = bad_val;
          // ASSERT_GT(bad_val, WipersCommand::ENABLE_CLEAN);
          break;
        case State::BLINKER:
          ret.blinker = bad_val;
          // ASSERT_GT(bad_val, VSC::BLINKER_HAZARD);
          break;
        case State::GEAR:
          ret.gear = bad_val;
          // ASSERT_GT(bad_val, VSC::GEAR_NEUTRAL);
          break;
        case State::HEADLIGHT:
          ret.headlight = bad_val;
          // ASSERT_GT(bad_val, HeadlightsCommand::ENABLE_HIGH);
          break;
        case State::MODE:
          ret.mode = bad_val;
          // ASSERT_GT(bad_val, VSR::MODE_DISENGAGED);
          break;
        default:
          throw std::logic_error{"Wrong state"};
      }
    }
    return ret;
  }

  bool8_t check(uint8_t result, uint8_t original, uint8_t max_value)
  {
    if (original > max_value) {
      EXPECT_NE(original, result);
      EXPECT_EQ(result, 0U);  // default to no command
    }
    return !HasFailure();
  }
};
// blinker, headlight, wiper, gear, and mode can be out of bounds
TEST_P(BadStateCommand, Basic)
{
  const auto param = GetParam();
  const auto state = make_bad_state(param);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl, state});
  EXPECT_EQ(ctrl, cmd.control());
  EXPECT_NE(cmd.state().value(), state);
  EXPECT_TRUE(has_report(StateMachineReport::BAD_STATE));
  // Check contents of state
  // flags
  {
    EXPECT_EQ(cmd.state()->hand_brake, state.hand_brake);
    EXPECT_EQ(cmd.state()->horn, state.horn);
  }
  // Actual ones with possible bad stuff
  // Unfortunately hardcoded--no way to enumerate over static variables
  EXPECT_TRUE(check(cmd.state()->wiper, state.wiper, WipersCommand::ENABLE_CLEAN));
  EXPECT_TRUE(check(cmd.state()->mode, state.mode, VSC::MODE_MANUAL));
  EXPECT_TRUE(check(cmd.state()->blinker, state.blinker, VSC::BLINKER_HAZARD));
  EXPECT_TRUE(check(cmd.state()->headlight, state.headlight, HeadlightsCommand::ENABLE_HIGH));
  EXPECT_TRUE(check(cmd.state()->gear, state.gear, VSC::GEAR_NEUTRAL));
}

/* TODO(Maxime CLEMENT): migrating from INSTANTIATE_TEST_CASE_P (deprecated) causes errors
INSTANTIATE_TEST_SUITE_P(
  Test,
  BadStateCommand,
  ::testing::Values(
    // Do combinatorial testing with N = 1
    BadStateCommandParam{std::vector<State>{State::WIPER}, 55U},
    BadStateCommandParam{std::vector<State>{State::BLINKER}, 53U},
    BadStateCommandParam{std::vector<State>{State::GEAR}, 50U},
    BadStateCommandParam{std::vector<State>{State::HEADLIGHT}, 65U},
    BadStateCommandParam{std::vector<State>{State::MODE}, 15U},
    // TODO(c.ho) more testing combinations
    // Everything
    BadStateCommandParam{
  std::vector<State>{State::MODE, State::WIPER, State::BLINKER, State::GEAR, State::HEADLIGHT},
  21U
}
    // cppcheck-suppress syntaxError
  ),
);
*/

// TODO(c.ho) bad state report test cases...
