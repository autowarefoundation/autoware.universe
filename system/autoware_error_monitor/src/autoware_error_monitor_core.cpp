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

#include <algorithm>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#define FMT_HEADER_ONLY
#include "fmt/format.h"

#include "autoware_error_monitor/autoware_error_monitor_core.hpp"

#include "autoware_error_monitor/diagnostics_filter.hpp"

namespace
{
int str2level(const std::string & level_str)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using std::regex_constants::icase;

  if (std::regex_match(level_str, std::regex("warn", icase))) {return DiagnosticStatus::WARN;}
  if (std::regex_match(level_str, std::regex("error", icase))) {return DiagnosticStatus::ERROR;}
  if (std::regex_match(level_str, std::regex("stale", icase))) {return DiagnosticStatus::STALE;}

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
}

bool isOverLevel(const int & diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return diag_level >= str2level(failure_level_str);
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus> & getTargetDiagnosticsRef(
  const int hazard_level, autoware_system_msgs::msg::HazardStatus * hazard_status)
{
  using autoware_system_msgs::msg::HazardStatus;

  if (hazard_level == HazardStatus::NO_FAULT) {return hazard_status->diagnostics_nf;}
  if (hazard_level == HazardStatus::SAFE_FAULT) {return hazard_status->diagnostics_sf;}
  if (hazard_level == HazardStatus::LATENT_FAULT) {return hazard_status->diagnostics_lf;}
  if (hazard_level == HazardStatus::SINGLE_POINT_FAULT) {return hazard_status->diagnostics_spf;}

  throw std::runtime_error(fmt::format("invalid hazard level: {}", hazard_level));
}
}  // namespace

AutowareErrorMonitor::AutowareErrorMonitor()
: Node("autoware_error_monitor"),
  update_rate_(declare_parameter("update_rate", 10)),
  ignore_missing_diagnostics_(declare_parameter("ignore_missing_diagnostics", false)),
  add_leaf_diagnostics_(declare_parameter("add_leaf_diagnostics", true))
{
  // Parameter
  loadRequiredModules(KeyName::autonomous_driving);
  loadRequiredModules(KeyName::remote_control);

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

void AutowareErrorMonitor::loadRequiredModules(const std::string & key)
{
  const auto param_key = std::string("required_modules.") + key + std::string(".names");

  this->declare_parameter(param_key);

  const auto names = this->get_parameter(param_key).as_string_array();
  if (names.size() == 0) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  RequiredModules required_modules;
  required_modules.reserve(names.size());

  DiagLevel default_level{{"sf_at", ""}, {"lf_at", ""}, {"spf_at", ""}};

  for (const auto & module_name : names) {
    const auto diag_level_key =
      std::string("required_modules.") + key + std::string(".diag_level.") + module_name;
    this->declare_parameters(diag_level_key, default_level);
    DiagLevel diag_level{};
    this->get_parameters(diag_level_key, diag_level);
    required_modules.emplace_back(module_name, diag_level);
  }

  required_modules_map_.insert(std::make_pair(key, required_modules));
}

void AutowareErrorMonitor::onDiagArray(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  diag_array_ = msg;

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

bool AutowareErrorMonitor::isDataReady()
{
  if (!diag_array_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for diag_array msg...");
    return false;
  }

  return true;
}

void AutowareErrorMonitor::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  autoware_system_msgs::msg::DrivingCapability driving_capability;

  driving_capability.autonomous_driving = judgeHazardStatus(KeyName::autonomous_driving);
  driving_capability.remote_control = judgeHazardStatus(KeyName::remote_control);

  pub_driving_capability_->publish(driving_capability);
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

int AutowareErrorMonitor::getHazardLevel(
  const DiagConfig & required_module, const int diag_level)
{
  using autoware_system_msgs::msg::HazardStatus;

  if (isOverLevel(diag_level, required_module.spf_at)) {return HazardStatus::SINGLE_POINT_FAULT;}
  if (isOverLevel(diag_level, required_module.lf_at)) {return HazardStatus::LATENT_FAULT;}
  if (isOverLevel(diag_level, required_module.sf_at)) {return HazardStatus::SAFE_FAULT;}

  return HazardStatus::NO_FAULT;
}

void AutowareErrorMonitor::appendHazardDiag(
  const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & hazard_diag,
  autoware_system_msgs::msg::HazardStatus * hazard_status)
{
  const auto hazard_level = getHazardLevel(required_module, hazard_diag.level);

  auto & target_diagnostics_ref = getTargetDiagnosticsRef(hazard_level, hazard_status);
  target_diagnostics_ref.push_back(hazard_diag);

  if (add_leaf_diagnostics_) {
    for (const auto & diag :
      diagnostics_filter::extractLeafChildrenDiagnostics(hazard_diag, diag_array_->status))
    {
      target_diagnostics_ref.push_back(diag);
    }
  }

  hazard_status->level = std::max(hazard_status->level, hazard_level);
}

autoware_system_msgs::msg::HazardStatus AutowareErrorMonitor::judgeHazardStatus(
  const std::string & key)
{
  using autoware_system_msgs::msg::HazardStatus;
  using diagnostic_msgs::msg::DiagnosticStatus;

  autoware_system_msgs::msg::HazardStatus hazard_status;

  for (const auto & required_module : required_modules_map_.at(key)) {
    const auto & diag_name = required_module.name;

    const auto latest_diag = getLatestDiag(diag_name);

    // no diag found
    if (!latest_diag) {
      if (!ignore_missing_diagnostics_) {
        DiagnosticStatus missing_diag;

        missing_diag.name = diag_name;
        missing_diag.hardware_id = "autoware_error_monitor";
        missing_diag.level = DiagnosticStatus::STALE;
        missing_diag.message = "no diag found";

        appendHazardDiag(required_module, missing_diag, &hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendHazardDiag(required_module, latest_diag->status, &hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = this->now() - latest_diag->header.stamp;
      if (time_diff.seconds() > diag_timeout_sec_) {
        DiagnosticStatus timeout_diag = latest_diag->status;
        timeout_diag.level = DiagnosticStatus::STALE;
        timeout_diag.message = "timeout";

        appendHazardDiag(required_module, timeout_diag, &hazard_status);
      }
    }
  }

  return hazard_status;
}
