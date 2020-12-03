// Copyright 2020 Tier IV, Inc.
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
#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include <autoware_error_monitor/autoware_error_monitor_core.hpp>

AutowareErrorMonitor::AutowareErrorMonitor()
: Node("autoware_error_monitor"),
  update_rate_(declare_parameter("update_rate", 10))
{
  // Parameter
  loadRequiredConditions(KeyName::manual_driving);
  loadRequiredConditions(KeyName::autonomous_driving);
  loadRequiredConditions(KeyName::remote_control);
  loadRequiredConditions(KeyName::safe_stop);
  loadRequiredConditions(KeyName::emergency_stop);

  // Subscriber
  sub_diag_array_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "input/diag_array", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onDiagArray, this, std::placeholders::_1));

  // Publisher
  pub_driving_capability_ = create_publisher<autoware_system_msgs::msg::DrivingCapability>(
    "output/driving_capability", rclcpp::QoS{1});

  // Timer
  auto timer_callback = std::bind(&AutowareErrorMonitor::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void AutowareErrorMonitor::loadRequiredConditions(const std::string & key)
{
  const auto param_key = std::string("required_conditions.") + key;

  this->declare_parameter(param_key);

  RequiredConditions value = this->get_parameter(param_key).as_string_array();
  if (value.size() == 0) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  required_conditions_map_.insert(std::make_pair(key, value));
}

void AutowareErrorMonitor::onDiagArray(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
      diag_buffer_map_.insert(std::make_pair(diag.name, DiagBuffer{}));
    }

    auto & diag_buffer = diag_buffer_map_.at(diag.name);
    diag_buffer.push_back(DiagStamped{header, diag});

    while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }
}

void AutowareErrorMonitor::onTimer()
{
  autoware_system_msgs::msg::DrivingCapability driving_capability;

  driving_capability.manual_driving = judgeCapability(KeyName::manual_driving);
  driving_capability.autonomous_driving = judgeCapability(KeyName::autonomous_driving);
  driving_capability.remote_control = judgeCapability(KeyName::remote_control);
  driving_capability.safe_stop = judgeCapability(KeyName::safe_stop);
  driving_capability.emergency_stop = judgeCapability(KeyName::emergency_stop);

  pub_driving_capability_->publish(driving_capability);
}

bool AutowareErrorMonitor::judgeCapability(const std::string & key)
{
  for (const std::string & required_condition : required_conditions_map_.at(key)) {
    const auto diag_name = fmt::format("/{}", required_condition);
    const auto & latest_diag = getLatestDiag(diag_name);

    if (!latest_diag) {
      return false;
    }

    if (latest_diag->status.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      return false;
    }

    const auto time_diff = rclcpp::Node::now() - latest_diag->header.stamp;
    if (time_diff.seconds() > diag_timeout_sec_) {
      return false;
    }
  }

  return true;
}

boost::optional<DiagStamped> AutowareErrorMonitor::getLatestDiag(const std::string & diag_name)
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }

  const auto & diag_buffer = diag_buffer_map_.at(diag_name);

  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}
