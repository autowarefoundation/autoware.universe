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
 * @file cpu_monitor_node.cpp
 * @brief CPU monitor node class
 */

#include <ros/ros.h>
#include <string>

#if defined _CPU_INTEL_
#include <system_monitor/cpu_monitor/intel_cpu_monitor.h>
#elif defined _CPU_ARM_
#include <system_monitor/cpu_monitor/arm_cpu_monitor.h>
#elif defined _CPU_RASPI_
#include <system_monitor/cpu_monitor/raspi_cpu_monitor.h>
#elif defined _CPU_TEGRA_
#include <system_monitor/cpu_monitor/tegra_cpu_monitor.h>
#else
#include <system_monitor/cpu_monitor/unknown_cpu_monitor.h>
#endif

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cpu_monitor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::shared_ptr<CPUMonitorBase> monitor;

  monitor = std::make_shared<CPUMonitor>(nh, pnh);

  monitor->getTempNames();
  monitor->getFreqNames();
  monitor->run();

  return 0;
}
