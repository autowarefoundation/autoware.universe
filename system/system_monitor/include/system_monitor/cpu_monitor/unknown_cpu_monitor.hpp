#ifndef SYSTEM_MONITOR_CPU_MONITOR_UNKNOWN_CPU_MONITOR_H
#define SYSTEM_MONITOR_CPU_MONITOR_UNKNOWN_CPU_MONITOR_H
/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file unknown_cpu_monitor.h
 * @brief Unknown CPU monitor class
 */

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

class CPUMonitor : public CPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  CPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options);
};

#endif  // SYSTEM_MONITOR_CPU_MONITOR_UNKNOWN_CPU_MONITOR_H
