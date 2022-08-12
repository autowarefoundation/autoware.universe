// Copyright 2022 TIER IV, Inc.
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

#include "diagnostics.hpp"

#include <string>
#include <vector>

namespace default_ad_api
{

using diagnostic_msgs::msg::DiagnosticStatus;

DiagnosticsMonitor::DiagnosticsMonitor(rclcpp::Node * node)
{
  const auto names = node->declare_parameter<std::vector<std::string>>("topic_monitor_names");
  for (const auto & name : names) {
    levels_[name] = DiagnosticStatus::STALE;
  }

  sub_diag_ = node->create_subscription<DiagnosticArray>(
    "/diagnostics", 50, std::bind(&DiagnosticsMonitor::on_diag, this, std::placeholders::_1));
}

bool DiagnosticsMonitor::is_ok()
{
  for (const auto & level : levels_) {
    if (level.second != DiagnosticStatus::OK) {
      return false;
    }
  }
  return true;
}

void DiagnosticsMonitor::on_diag(const DiagnosticArray::ConstSharedPtr msg)
{
  const auto is_target = [](const std::string & name) {
    const std::string suffix = "default_ad_api";
    if (name.size() < suffix.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), name.rbegin());
  };

  for (const auto & status : msg->status) {
    if (status.hardware_id == "topic_state_monitor") {
      if (is_target(status.name)) {
        levels_[status.name] = status.level;
      }
    }
  }
}

}  // namespace default_ad_api
