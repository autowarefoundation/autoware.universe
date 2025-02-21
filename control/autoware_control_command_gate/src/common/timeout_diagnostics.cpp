// Copyright 2025 The Autoware Contributors
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

#include "timeout_diagnostics.hpp"

#include <string>

namespace autoware::control_command_gate
{

TimeoutDiag::TimeoutDiag(
  const Params & params, const rclcpp::Clock & clock, const std::string & name)
: DiagnosticTask(name), params_(params), clock_(clock)
{
}

void TimeoutDiag::update()
{
  const auto now = clock_.now();
  if (last_stamp_) {
    duration_ = (now - *last_stamp_).seconds();
  }
  last_stamp_ = now;
}

void TimeoutDiag::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (!duration_) {
    stat.summary(DiagnosticStatus::ERROR, "no data");
    return;
  }

  if (*duration_ > params_.error_duration_) {
    stat.summary(DiagnosticStatus::ERROR, "timeout");
  } else if (*duration_ > params_.warn_duration_) {
    stat.summary(DiagnosticStatus::WARN, "late");
  } else {
    stat.summary(DiagnosticStatus::OK, "OK");
  }
  stat.add("duration", *duration_);
}

}  // namespace autoware::control_command_gate
