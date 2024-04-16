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

/**
 * @file _voltage_monitor.cpp
 * @brief  voltage monitor class
 */

#include "system_monitor/voltage_monitor/voltage_monitor.hpp"

#include "system_monitor/msr_reader/msr_reader.hpp"
#include "system_monitor/system_monitor_utility.hpp"

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace bp = boost::process;

VoltageMonitor::VoltageMonitor(const rclcpp::NodeOptions & options)
: Node("voltage_monitor", options),
  updater_(this),
  hostname_(),
  voltage_timeout_(declare_parameter<int>("voltage_timeout", 5)),
  battery_timeout_(declare_parameter<int>("battery_timeout", 5)),
  voltage_elapsed_ms_(0),
  battery_elapsed_ms_(0)
{
  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  voltage_string_ = declare_parameter<std::string>("cmos_battery_label", "");
  voltage_warn_ = declare_parameter<float>("cmos_battery_warn", 2.95);
  voltage_error_ = declare_parameter<float>("cmos_battery_error", 2.75);
  sensors_exists_ = false;
  if (voltage_string_ == "") {
    sensors_exists_ = false;
  } else {
    // Check if command exists
    fs::path p = bp::search_path("sensors");
    sensors_exists_ = (p.empty()) ? false : true;
  }
  gethostname(hostname_, sizeof(hostname_));
  auto callback = &VoltageMonitor::checkBatteryStatus;
  if (sensors_exists_) {
    try {
      std::regex re(R"((\d+).(\d+))");
      voltage_regex_ = re;
    } catch (std::regex_error & e) {
      // never comes here.
      RCLCPP_WARN(get_logger(), "std::regex_error %d", e.code());
      return;
    }
    callback = &VoltageMonitor::checkVoltage;
  }
  updater_.add("CMOS Battery Status", this, callback);

  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1), std::bind(&VoltageMonitor::onTimer, this),
    timer_callback_group_);
}

void VoltageMonitor::checkVoltage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  float tmp_voltage = 0.0;
  std::string tmp_sensors_error_str;
  std::string tmp_format_error_str;
  std::string tmp_pipe2_err_str;
  double tmp_elapsed_ms;

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(voltage_mutex_);
    tmp_voltage = voltage_;
    tmp_sensors_error_str = sensors_error_str_;
    tmp_format_error_str = format_error_str_;
    tmp_pipe2_err_str = pipe2_err_str_;
    tmp_elapsed_ms = voltage_elapsed_ms_;
  }

  if (!tmp_pipe2_err_str.empty()) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", tmp_pipe2_err_str);
    return;
  }

  if (!tmp_sensors_error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "sensors error");
    stat.add("sensors", tmp_sensors_error_str);
    return;
  }

  if (!tmp_format_error_str.empty()) {
    stat.summary(DiagStatus::WARN, "format error");
    stat.add("exception in std::regex_search", tmp_format_error_str);
    return;
  }

  stat.add("CMOS battery voltage", fmt::format("{}", tmp_voltage));

  if (tmp_elapsed_ms > voltage_timeout_ * 1000) {
    stat.summary(DiagStatus::WARN, "sensors timeout expired");
  } else if (tmp_voltage == 0.0) {
    stat.summary(DiagStatus::WARN, "read voltage error");
  } else if (tmp_voltage < voltage_error_) {
    stat.summary(DiagStatus::WARN, "Battery Died");
  } else if (tmp_voltage < voltage_warn_) {
    stat.summary(DiagStatus::WARN, "Low Battery");
  } else {
    stat.summary(DiagStatus::OK, "OK");
  }

  stat.addf("execution time", "%f ms", tmp_elapsed_ms);
}

void VoltageMonitor::readVoltageStatus(
  float & tmp_voltage, std::string & tmp_sensors_error_str, std::string & tmp_format_error_str,
  std::string & tmp_pipe2_err_str)
{
  int out_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(out_fd, O_CLOEXEC) != 0)) {
    tmp_pipe2_err_str = std::string(strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (RCUTILS_UNLIKELY(pipe2(err_fd, O_CLOEXEC) != 0)) {
    tmp_pipe2_err_str = std::string(strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("sensors", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (RCUTILS_UNLIKELY(c.exit_code() != 0)) {  // failed to execute sensors
    std::ostringstream os;
    is_err >> os.rdbuf();
    tmp_sensors_error_str = os.str().c_str();
    return;
  }
  std::string line;
  while (std::getline(is_out, line)) {
    auto voltageStringPos = line.find(voltage_string_.c_str());
    if (voltageStringPos != std::string::npos) {
      try {
        std::smatch match;
        std::regex_search(line, match, voltage_regex_);
        auto voltageString = match.str();
        tmp_voltage = std::stof(voltageString);
      } catch (std::regex_error & e) {
        tmp_format_error_str = fmt::format("{}", e.code());
        return;
      }
      break;
    }
  }
}

void VoltageMonitor::checkBatteryStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::string tmp_ifstream_error_str;
  bool tmp_status = false;
  double tmp_elapsed_ms = 0.0;

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    tmp_status = status_;
    tmp_ifstream_error_str = ifstream_error_str_;
    tmp_elapsed_ms = battery_elapsed_ms_;
  }

  if (!tmp_ifstream_error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "ifstream error");
    stat.add("std::ifstream", tmp_ifstream_error_str);
    return;
  }

  if (tmp_elapsed_ms > battery_timeout_ * 1000) {
    stat.add("CMOS battery status", std::string("reading battery status timeout expired"));
    stat.summary(DiagStatus::WARN, "reading battery status timeout expired");
  } else if (tmp_elapsed_ms == 0.0) {
    stat.add("CMOS battery status", std::string("reading battery status error"));
    stat.summary(DiagStatus::WARN, "reading battery status error");
  } else if (tmp_status) {
    stat.add("CMOS battery status", std::string("OK"));
    stat.summary(DiagStatus::OK, "OK");
  } else {
    stat.add("CMOS battery status", std::string("Battery Dead"));
    stat.summary(DiagStatus::WARN, "Battery Dead");
  }

  stat.addf("execution time", "%f ms", tmp_elapsed_ms);
}

void VoltageMonitor::readBatteryStatus(bool & tmp_status, std::string & tmp_ifstream_error_str)
{
  // Get status of RTC
  std::ifstream ifs("/proc/driver/rtc");
  if (!ifs) {
    tmp_ifstream_error_str = "Error opening /proc/driver/rtc";
    return;
  }

  std::string line;
  bool status = false;
  while (std::getline(ifs, line)) {
    auto batStatusLine = line.find("batt_status");
    if (batStatusLine != std::string::npos) {
      auto batStatus = line.find("okay");
      if (batStatus != std::string::npos) {
        tmp_status = true;
        break;
      }
    }
  }
}

void VoltageMonitor::onTimer()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // read voltage status
  if (sensors_exists_)
  {
    stop_watch.tic("execution_time");

    std::string tmp_sensors_error_str;
    std::string tmp_format_error_str;
    std::string tmp_pipe2_err_str;
    float tmp_voltage = 0.0;

    readVoltageStatus(tmp_voltage, tmp_sensors_error_str, tmp_format_error_str, tmp_pipe2_err_str);

    double tmp_elapsed_ms = stop_watch.toc("execution_time");
    // thread-safe copy
    {
      std::lock_guard<std::mutex> lock(voltage_mutex_);
      voltage_ = tmp_voltage;
      sensors_error_str_ = tmp_sensors_error_str;
      format_error_str_ = tmp_format_error_str;
      pipe2_err_str_ = tmp_pipe2_err_str;
      voltage_elapsed_ms_ = tmp_elapsed_ms;
    }
  } else {
  // read battery 
    stop_watch.tic("execution_time");

    std::string tmp_ifstream_error_str;
    bool tmp_status = false;

    readBatteryStatus(tmp_status, tmp_ifstream_error_str);

    double tmp_elapsed_ms = stop_watch.toc("execution_time");
    // thread-safe copy
    {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      status_ = tmp_status;
      ifstream_error_str_ = tmp_ifstream_error_str;
      battery_elapsed_ms_ = tmp_elapsed_ms;
    }
  }
}

void VoltageMonitor::update()
{
  updater_.force_update();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoltageMonitor)
