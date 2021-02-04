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

/**
 * @file velodyne_monitor.cpp
 * @brief Velodyne monitor class
 */

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "velodyne_monitor/velodyne_monitor.hpp"

#include "boost/algorithm/string/join.hpp"

#define FMT_HEADER_ONLY
#include "fmt/format.h"

VelodyneMonitor::VelodyneMonitor()
: Node("velodyne_monitor"),
  updater_(this)
{
  timeout_ = declare_parameter("timeout", 0.5);
  ip_address_ = declare_parameter("ip_address", "192.168.1.201");
  temp_cold_warn_ = declare_parameter("temp_cold_warn", -5.0);
  temp_cold_error_ = declare_parameter("temp_cold_error", -10.0);
  temp_hot_warn_ = declare_parameter("temp_hot_warn", 75.0);
  temp_hot_error_ = declare_parameter("temp_hot_error", 80.0);
  rpm_ratio_warn_ = declare_parameter("rpm_ratio_warn", 0.80);
  rpm_ratio_error_ = declare_parameter("rpm_ratio_error", 0.70);

  updater_.add("velodyne_connection", this, &VelodyneMonitor::checkConnection);
  updater_.add("velodyne_temperature", this, &VelodyneMonitor::checkTemperature);
  updater_.add("velodyne_rpm", this, &VelodyneMonitor::checkMotorRpm);

  auto config = client::http_client_config();
  int timeout = timeout_ * 1000;
  config.set_timeout(std::chrono::milliseconds(timeout));

  // Creates a new http_client connected to specified uri
  client_.reset(new client::http_client("http://" + ip_address_, config));

  updater_.setHardwareID("velodyne");

  // Timer
  auto timer_callback = std::bind(&VelodyneMonitor::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void VelodyneMonitor::checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  http::http_response res;
  std::string err_msg = "";

  // Sends an HTTP-GET request
  if (!requestGET("/cgi/info.json", res, err_msg)) {
    stat.summary(DiagStatus::ERROR, err_msg);
    return;
  }

  // Extracts the body of the request message into a json value
  info_json_ = res.extract_json().get();

  updater_.setHardwareIDf(
    "%s: %s", info_json_["model"].as_string().c_str(), info_json_["serial"].as_string().c_str());

  stat.summary(DiagStatus::OK, "OK");
}

void VelodyneMonitor::checkTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level_top = DiagStatus::OK;
  int level_bot = DiagStatus::OK;
  std::vector<std::string> msg;
  std::string err_msg = "";

  http::http_response res;
  diag_json_received_ = false;

  // Sends an HTTP-GET request
  if (!requestGET("/cgi/diag.json", res, err_msg)) {
    stat.summary(DiagStatus::ERROR, err_msg);
    return;
  }

  diag_json_received_ = true;

  // Extracts the body of the request message into a json value
  diag_json_ = res.extract_json().get();

  const float top_temp =
    convertTemperature(diag_json_["volt_temp"]["top"]["lm20_temp"].as_integer());
  const float bot_temp =
    convertTemperature(diag_json_["volt_temp"]["bot"]["lm20_temp"].as_integer());

  // Check top board temperature
  if (top_temp < temp_cold_error_) {
    level_top = DiagStatus::ERROR;
    msg.emplace_back("Top board temperature too cold");
  } else if (top_temp < temp_cold_warn_) {
    level_top = DiagStatus::WARN;
    msg.emplace_back("Top board temperature cold");
  } else if (top_temp > temp_hot_error_) {
    level_top = DiagStatus::ERROR;
    msg.emplace_back("Top board temperature too hot");
  } else if (top_temp > temp_hot_warn_) {
    level_top = DiagStatus::WARN;
    msg.emplace_back("Top board temperature hot");
  }

  // Check bottom board temperature
  if (bot_temp < temp_cold_error_) {
    level_bot = DiagStatus::ERROR;
    msg.emplace_back("Bottom board temperature too cold");
  } else if (bot_temp < temp_cold_warn_) {
    level_bot = DiagStatus::WARN;
    msg.emplace_back("Bottom board temperature cold");
  } else if (bot_temp > temp_hot_error_) {
    level_bot = DiagStatus::ERROR;
    msg.emplace_back("Bottom board temperature too hot");
  } else if (bot_temp > temp_hot_warn_) {
    level_bot = DiagStatus::WARN;
    msg.emplace_back("Bottom board temperature hot");
  }

  stat.addf("Top board", "%.3lf DegC", top_temp);
  stat.addf("Bottom board", "%.3lf DegC", bot_temp);

  if (msg.empty()) {msg.emplace_back("OK");}

  stat.summary(std::max(level_top, level_bot), boost::algorithm::join(msg, ", "));
}

void VelodyneMonitor::checkMotorRpm(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  http::http_response res;
  std::string err_msg = "";

  // Sends an HTTP-GET request
  if (!requestGET("/cgi/settings.json", res, err_msg)) {
    stat.summary(DiagStatus::ERROR, err_msg);
    return;
  }

  // Extracts the body of the request message into a json value
  settings_json_ = res.extract_json().get();

  // Sends an HTTP-GET request
  if (!requestGET("/cgi/status.json", res, err_msg)) {
    stat.summary(DiagStatus::ERROR, err_msg);
    return;
  }

  // Extracts the body of the request message into a json value
  status_json_ = res.extract_json().get();

  const double setting = settings_json_["rpm"].as_double();
  const double rpm = status_json_["motor"]["rpm"].as_double();
  const double ratio = rpm / setting;

  if (ratio < rpm_ratio_error_) {
    level = DiagStatus::ERROR;
  } else if (ratio < rpm_ratio_warn_) {
    level = DiagStatus::WARN;
  }

  stat.addf("Ratio", "%.2lf %%", ratio * 1e2);
  stat.addf("Setting", "%.0f", setting);
  stat.addf("Current", "%.0f", rpm);

  stat.summary(level, rpm_dict_.at(level));
}

bool VelodyneMonitor::requestGET(
  const std::string & path_query, http::http_response & res, std::string & err_msg)
{
  // Asynchronously sends an HTTP request
  try {
    res = client_->request(http::methods::GET, path_query).get();
  } catch (const std::exception & e) {
    err_msg = e.what();
    return false;
  }

  if (res.status_code() != web::http::status_codes::OK) {
    err_msg = fmt::format("{}: {}", res.status_code(), res.reason_phrase().c_str());
    return false;
  }

  return true;
}

float VelodyneMonitor::convertTemperature(int raw)
{
  return std::sqrt(2.1962e6 + (1.8639 - static_cast<float>(raw) * 5.0 / 4096) / 3.88e-6) - 1481.96;
}

void VelodyneMonitor::onTimer() {updater_.force_update();}
