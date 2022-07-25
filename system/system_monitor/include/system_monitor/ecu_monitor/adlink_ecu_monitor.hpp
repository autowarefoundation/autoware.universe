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
 * @file adlink_ecu_monitor.h
 * @brief  ECU monitor class
 */

#ifndef SYSTEM_MONITOR__ECU_MONITOR__ADLINK_ECU_MONITOR_HPP_
#define SYSTEM_MONITOR__ECU_MONITOR__ADLINK_ECU_MONITOR_HPP_

#include "system_monitor/ecu_monitor/ecu_monitor_base.hpp"

#include <string>

class ECUMonitor : public ECUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit ECUMonitor(const rclcpp::NodeOptions & options);

protected:
  /**
   * @brief check ECU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkVoltage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)
  bool sensors_exists_;                      //!< @brief flag if sensors exists
  float voltage_warn_;
  float voltage_error_;
};

#endif  // SYSTEM_MONITOR__ECU_MONITOR__ADLINK_ECU_MONITOR_HPP_
