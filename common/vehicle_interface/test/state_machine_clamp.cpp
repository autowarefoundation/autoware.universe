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

#include <limits>

#include "state_machine.hpp"

using autoware::common::types::bool8_t;

struct CommandClampParam
{
  decltype(VCC::long_accel_mps2) accel;
  decltype(VCC::front_wheel_angle_rad) front_steer;
  bool8_t expect_warn;
};
// TODO(c.ho) use config struct instead
constexpr decltype(CommandClampParam::accel) max_accel{3.0F};
constexpr decltype(CommandClampParam::accel) min_accel{-3.0F};
constexpr decltype(CommandClampParam::accel) accel_threshold{1.0F};
constexpr decltype(CommandClampParam::front_steer) max_front_steer{0.331F};
constexpr decltype(CommandClampParam::front_steer) min_front_steer{-0.331F};
constexpr decltype(CommandClampParam::front_steer) front_steer_threshold{0.3F};


class CommandClamp : public state_machine, public ::testing::WithParamInterface<CommandClampParam>
{
  void SetUp()
  {
    // Check positive threshold, big enough, symmetric limits
    ASSERT_GT(accel_threshold, decltype(accel_threshold) {});
    ASSERT_GT(accel_threshold, std::numeric_limits<decltype(max_accel)>::epsilon());
    ASSERT_FLOAT_EQ(-max_accel, min_accel);
    ASSERT_GT(front_steer_threshold, decltype(front_steer_threshold) {});
    ASSERT_GT(front_steer_threshold, std::numeric_limits<decltype(max_front_steer)>::epsilon());
    ASSERT_FLOAT_EQ(-max_front_steer, min_front_steer);
  }
};

TEST_P(CommandClamp, Basic)
{
  const auto param = GetParam();
  // Make command, apply
  const auto ctrl =
    VCC{}.set__long_accel_mps2(param.accel).set__front_wheel_angle_rad(param.front_steer);
  const auto cmd = sm_.compute_safe_commands(Command{ctrl});
  if (param.accel >= 0.0F) {  // Check is in case you run into automatic gear shift logic
    EXPECT_FALSE(cmd.state());
  }
  // Ensure resulting command is not outside of bounds
  EXPECT_LE(cmd.control().long_accel_mps2, max_accel);  // TODO(c.ho) use config struct instead
  EXPECT_GE(cmd.control().long_accel_mps2, min_accel);
  EXPECT_LE(cmd.control().front_wheel_angle_rad, max_front_steer);
  EXPECT_GE(cmd.control().front_wheel_angle_rad, min_front_steer);

  if (param.expect_warn) {
    EXPECT_TRUE(has_report(StateMachineReport::CLAMP_PAST_THRESHOLD));
  } else {
    EXPECT_TRUE(sm_.reports().empty());
  }
}

/* TODO(Maxime CLEMENT): migrating from INSTANTIATE_TEST_CASE_P (deprecated) causes errors
INSTANTIATE_TEST_SUITE_P(
  Test,
  CommandClamp,
  ::testing::Values(
    // Accel
    CommandClampParam{max_accel, 0.0F, false},
    CommandClampParam{max_accel + std::numeric_limits<decltype(max_accel)>::epsilon(), 0.0F, false},
    CommandClampParam{max_accel + std::numeric_limits<decltype(max_accel)>::epsilon() +
      accel_threshold, 0.0F, true},
    CommandClampParam{min_accel, 0.0F, false},
    CommandClampParam{min_accel - std::numeric_limits<decltype(min_accel)>::epsilon(), 0.0F, false},
    CommandClampParam{min_accel - std::numeric_limits<decltype(min_accel)>::epsilon() -
      accel_threshold, 0.0F, true},
    // Front steer
    CommandClampParam{0.0F, max_front_steer, false},
    CommandClampParam{0.0F,
      max_front_steer + std::numeric_limits<decltype(max_accel)>::epsilon(), false},
    CommandClampParam{0.0F, max_front_steer +
      std::numeric_limits<decltype(max_front_steer)>::epsilon() + front_steer_threshold, true},
    CommandClampParam{0.0F, min_front_steer, false},
    CommandClampParam{0.0F,
      min_front_steer - std::numeric_limits<decltype(min_accel)>::epsilon(), false},
    CommandClampParam{0.0F, min_front_steer -
      std::numeric_limits<decltype(min_front_steer)>::epsilon() - front_steer_threshold, true}
    // cppcheck-suppress syntaxError
  ),
);
*/
