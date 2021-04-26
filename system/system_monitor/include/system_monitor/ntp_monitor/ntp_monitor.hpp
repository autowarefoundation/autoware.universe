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
 * @file ntp_monitor.h
 * @brief NTP monitor class
 */

#ifndef SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_
#define SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_

#include <climits>
#include <map>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"

class NTPMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  NTPMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check NTP Offset
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkOffset(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief thread function to execute ntpdate
   */
  void executeNtpdate();

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name
  bool ntpdate_exists_;               //!< @brief flag if ntpdate exists

  std::string server_;  //!< @brief Reference server
  float offset_warn_;   //!< @brief NTP offset(us) to generate warning
  float offset_error_;  //!< @brief NTP offset(us) to generate error

  std::thread thread_;  //!< @brief thread to execute ntpdate
  std::string error_;   //!< @brief error output of ntpdate
  float offset_;        //!< @brief NTP offset(us)
  float delay_;         //!< @brief NTP delay(us)

  /**
   * @brief NTP offset status messages
   */
  const std::map<int, const char *> offset_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high"}, {DiagStatus::ERROR, "too high"}};
};

#endif  // SYSTEM_MONITOR__NTP_MONITOR__NTP_MONITOR_HPP_
