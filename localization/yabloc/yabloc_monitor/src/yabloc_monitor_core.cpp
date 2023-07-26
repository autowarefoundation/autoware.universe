// Copyright 2023 TIER IV
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

#include "yabloc_monitor_core.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;

YabLocMonitor::YabLocMonitor() : Node("yabloc_monitor")
{
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(100).period(), std::bind(&YabLocMonitor::timer_callback, this));

  // Evaluation moduels
  availability_module_ = std::make_unique<AvailabilityModule>(this);
}

void YabLocMonitor::timer_callback()
{
  diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
  diag_status_msg.name = "yabloc";
  diag_status_msg.hardware_id = "";

  diagnostic_msgs::msg::KeyValue key_value_msg;

  // Check availability
  key_value_msg.key = "availability";
  key_value_msg.value = availability_module_->is_available();
  diag_status_msg.values.push_back(key_value_msg);

  // Update total diag status of YabLoc
  diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  // Publish msg
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_status_msg);
  diagnostics_pub_->publish(diag_msg);
}
