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
 * @file ecu_monitor_base.h
 * @brief ECU monitor base class
 */

#ifndef SYSTEM_MONITOR__ECU_MONITOR__ECU_MONITOR_BASE_HPP_
#define SYSTEM_MONITOR__MONITOR__ECU_MONITOR_BASE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <map>
#include <string>
#include <vector>

class ECUMonitorBase : public rclcpp::Node
{
public:
  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  ECUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];        //!< @brief host name
};

#endif  // SYSTEM_MONITOR_ECU_MONITOR_ECU_MONITOR_BASE_HPP_
