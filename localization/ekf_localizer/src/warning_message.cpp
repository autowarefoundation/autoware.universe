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

#include <string>

#include <fmt/core.h>

std::string poseDelayStepWarningMessage(
  const double delay_time, const double extend_state_step, const double ekf_dt)
{
  const std::string s = "Pose delay exceeds the compensation limit, ignored. "
                        "delay: {:.3f}[s], limit = extend_state_step * ekf_dt : {:.3f}[s]";
  return fmt::format(s, delay_time, extend_state_step * ekf_dt);
}

std::string twistDelayStepWarningMessage(
  const double delay_time, const double extend_state_step, const double ekf_dt)
{
  const std::string s = "Twist delay exceeds the compensation limit, ignored. "
                        "delay: {:.3f}[s], limit = extend_state_step * ekf_dt : {:.3f}[s]";
  return fmt::format(s, delay_time, extend_state_step * ekf_dt);
}
