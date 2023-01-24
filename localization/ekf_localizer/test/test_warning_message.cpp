// Copyright 2022 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ekf_localizer/warning_message.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

TEST(PoseDelayStepWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    poseDelayStepWarningMessage(6.0, 2.5, 2.0).c_str(),
    "Pose delay exceeds the compensation limit, ignored. "
    "delay: 6.000[s], limit = extend_state_step * ekf_dt : 5.000[s]");
}

TEST(TwistDelayStepWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    twistDelayStepWarningMessage(10.0, 3.5, 2.0).c_str(),
    "Twist delay exceeds the compensation limit, ignored. "
    "delay: 10.000[s], limit = extend_state_step * ekf_dt : 7.000[s]");
}
