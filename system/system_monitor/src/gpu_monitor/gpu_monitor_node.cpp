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
 * @file gpu_monitor_node.cpp
 * @brief GPU monitor node class
 */

#include <ros/ros.h>
#include <string>

#if defined _GPU_NVML_
#include <system_monitor/gpu_monitor/nvml_gpu_monitor.h>
#elif defined _GPU_TEGRA_
#include <system_monitor/gpu_monitor/tegra_gpu_monitor.h>
#else
#include <system_monitor/gpu_monitor/unknown_gpu_monitor.h>
#endif

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gpu_monitor");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::shared_ptr<GPUMonitorBase> monitor;

  monitor = std::make_shared<GPUMonitor>(nh, pnh);

  monitor->run();

  return 0;
}
