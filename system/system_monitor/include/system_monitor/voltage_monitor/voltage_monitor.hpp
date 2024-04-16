// Copyright 2022 Tier IV, Inc.
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
 * @file voltage_monitor.h
 * @brief  voltage monitor class
 */

#ifndef SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_
#define SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <regex>
#include <string>
class VoltageMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit VoltageMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  /**
   * @brief check CMOS battery
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkVoltage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)
  /**
   * @brief Timer callback to execute sensors command
   */
  void readVoltageStatus(float & tmp_voltage, std::string & tmp_sensors_error_str, std::string & tmp_format_error_str, std::string & tmp_pipe2_err_str);

  /**
   * @brief check CMOS battery
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkBatteryStatus(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)
  /**
   * @brief Timer callback to reading battery status
   */
  void readBatteryStatus(bool & tmp_status, std::string & tmp_ifstream_error_str);

  /**
   * @brief Timer callback
   */
  void onTimer();

  float voltage_warn_;
  float voltage_error_;
  std::string voltage_string_;
  std::regex voltage_regex_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::mutex voltage_mutex_;
  float voltage_;
  int voltage_timeout_;
  std::string sensors_error_str_;
  std::string format_error_str_;
  std::string pipe2_err_str_;
  double voltage_elapsed_ms_;

  std::mutex battery_mutex_;
  bool status_;
  int battery_timeout_;
  std::string ifstream_error_str_;
  double battery_elapsed_ms_;
<<<<<<< HEAD
  std::mutex battery_timeout_mutex_;
  bool battery_timeout_expired_;
=======

  bool sensors_exists_;
>>>>>>> 07ac8c080 (feat: fix timeout values)
};

#endif  // SYSTEM_MONITOR__VOLTAGE_MONITOR__VOLTAGE_MONITOR_HPP_
