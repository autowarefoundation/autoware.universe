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

#include "ekf_localizer/check.hpp"

#include "ekf_localizer/warning.hpp"

#include <fmt/core.h>

#include <string>

bool checkDelayStep(const Warning & warning, const int delay_step, const int extend_state_step)
{
  const bool good = delay_step < extend_state_step;
  if (!good) {
    const std::string s = "The delay step {} should be less than the maximum state step {}.";
    warning.warnThrottle(fmt::format(s, delay_step, extend_state_step), 1000);
  }
  return good;
}
