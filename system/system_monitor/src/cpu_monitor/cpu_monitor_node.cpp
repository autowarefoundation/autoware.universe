// Copyright 2020 Autoware Foundation
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
 * @file cpu_monitor_node.cpp
 * @brief CPU monitor node class
 */

#include <rclcpp/rclcpp.hpp>

#if defined _CPU_INTEL_
#include <system_monitor/cpu_monitor/intel_cpu_monitor.hpp>
#elif defined _CPU_ARM_
#include <system_monitor/cpu_monitor/arm_cpu_monitor.hpp>
#elif defined _CPU_RASPI_
#include <system_monitor/cpu_monitor/raspi_cpu_monitor.hpp>
#elif defined _CPU_TEGRA_
#include <system_monitor/cpu_monitor/tegra_cpu_monitor.hpp>
#else
#include <system_monitor/cpu_monitor/unknown_cpu_monitor.hpp>
#endif
#include <system_monitor/utils.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<CPUMonitorBase> monitor = std::make_shared<CPUMonitor>("cpu_monitor", options);

  monitor->getTempNames();
  monitor->getFreqNames();

  spin_and_update(monitor, std::chrono::seconds(1U));

  rclcpp::shutdown();
  return 0;
}
