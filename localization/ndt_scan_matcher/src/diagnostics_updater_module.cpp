// Copyright 2023 Autoware Foundation
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

#include "ndt_scan_matcher/diagnostics_updater_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

DiagnosticsUpdaterModule::DiagnosticsUpdaterModule(
  rclcpp::Node * node, const double period, const std::string & prefix_diagnostic_name)
{
  node_.reset(node);
  if (node_ == nullptr) {
    return;  // TODO throw error
  }

  publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  update_timer_ = rclcpp::create_timer(
    node_, node_->get_clock(), rclcpp::Duration::from_seconds(period),
    std::bind(&DiagnosticsUpdaterModule::update, this));

  prefix_diagnostic_name_ =
    prefix_diagnostic_name.empty() ? "" : (prefix_diagnostic_name + std::string(": "));
}

void DiagnosticsUpdaterModule::publish(
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec)
{
  for (std::vector<diagnostic_msgs::msg::DiagnosticStatus>::iterator iter = status_vec.begin();
       iter != status_vec.end(); iter++) {
    iter->name =
      prefix_diagnostic_name_ + std::string(node_->get_name()) + std::string(": ") + iter->name;
  }
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.status = status_vec;
  msg.header.stamp = node_->now();
  publisher_->publish(msg);
}

void DiagnosticsUpdaterModule::update()
{
  if (rclcpp::ok()) {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

    std::unique_lock<std::mutex> lock(
      lock_);  // Make sure no adds happen while we are processing here.
    const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
    for (std::vector<DiagnosticTaskInternal>::const_iterator iter = tasks.begin();
         iter != tasks.end(); iter++) {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = iter->getName();
      status.level = 2;
      status.message = "No message was set";
      status.hardware_id = node_->get_name();

      iter->run(status);

      if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        status.message = "OK";
      }

      status_vec.push_back(status);
    }
    publish(status_vec);
  }
}
