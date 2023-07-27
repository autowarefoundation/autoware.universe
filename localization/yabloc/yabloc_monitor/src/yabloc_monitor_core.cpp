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

#include <memory>

using std::placeholders::_1;

YabLocMonitor::YabLocMonitor() : Node("yabloc_monitor"), updater_(this)
{
  updater_.setHardwareID(get_name());
  updater_.add("yabloc_status", this, &YabLocMonitor::update_diagnostics);

  // Set timer
  using std::chrono_literals::operator""ms;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&YabLocMonitor::on_timer, this));

  // Evaluation moduels
  availability_module_ = std::make_unique<AvailabilityModule>(this);
}

void YabLocMonitor::on_timer()
{
  updater_.force_update();
}

void YabLocMonitor::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("Availability", availability_module_->is_available() ? "OK" : "NG");
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");  // TODO
}
