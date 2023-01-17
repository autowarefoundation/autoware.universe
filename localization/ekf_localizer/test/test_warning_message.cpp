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

TEST(DelayStepWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    delayStepWarningMessage(4.0, 3.5).c_str(),
    "The delay step 4 should be less than the maximum state step 3.5.");
}
