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
 * @file nvml_gpu_monitor.cpp
 * @brief GPU monitor class
 */

#include <algorithm>
#include <string>
#include <vector>

#include "boost/algorithm/string.hpp"

#include "fmt/format.h"

#include "system_monitor/gpu_monitor/nvml_gpu_monitor.hpp"

GPUMonitor::GPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
: GPUMonitorBase(node_name, options)
{
  // Include frequency into GPU Thermal Throttling thus remove.
  updater_.removeByName("GPU Frequency");

  nvmlReturn_t ret;

  ret = nvmlInit();
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize NVML: %s\n", nvmlErrorString(ret));
  }

  unsigned int deviceCount = 0;
  ret = nvmlDeviceGetCount(&deviceCount);
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the number of compute devices: %s",
      nvmlErrorString(ret));
  }

  for (int index = 0; index < deviceCount; ++index) {
    gpu_info info;
    ret = nvmlDeviceGetHandleByIndex(index, &info.device);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to acquire the handle for a particular device [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetName(info.device, info.name, NVML_DEVICE_NAME_BUFFER_SIZE);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the name of this device [%d]: %s", index, nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetPciInfo(info.device, &info.pci);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the PCI attributes [%d]: %s", index, nvmlErrorString(ret));
      continue;
    }
    gpus_.push_back(info);
  }
}

void GPUMonitor::shut_down()
{
  nvmlReturn_t ret = nvmlShutdown();
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to shut down NVML: %s", nvmlErrorString(ret));
  }
}

void GPUMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret;

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    unsigned int temp = 0;
    ret = nvmlDeviceGetTemperature(itr->device, NVML_TEMPERATURE_GPU, &temp);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current temperature");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    level = DiagStatus::OK;
    stat.addf(itr->name, "%d.0 DegC", temp);
    if (temp >= temp_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  stat.summary(level, temp_dict_.at(level));
}

void GPUMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret;

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    ret = nvmlDeviceGetUtilizationRates(itr->device, &itr->utilization);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current utilization rates");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    level = DiagStatus::OK;
    float usage = static_cast<float>(itr->utilization.gpu) / 100.0;
    if (usage >= gpu_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (usage >= gpu_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%d.0%%", itr->utilization.gpu);

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, load_dict_.at(whole_level));
}

void GPUMonitor::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret;

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    nvmlMemory_t memory;
    ret = nvmlDeviceGetMemoryInfo(itr->device, &memory);
    if (ret != NVML_SUCCESS) {
      stat.summary(
        DiagStatus::ERROR, "Failed to retrieve the amount of used, free and total memory");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    level = DiagStatus::OK;
    float usage = static_cast<float>(itr->utilization.memory) / 100.0;
    if (usage >= memory_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (usage >= memory_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%d.0%%", itr->utilization.memory);
    stat.add(fmt::format("GPU {}: total", index), toHumanReadable(memory.total));
    stat.add(fmt::format("GPU {}: used", index), toHumanReadable(memory.used));
    stat.add(fmt::format("GPU {}: free", index), toHumanReadable(memory.free));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, load_dict_.at(whole_level));
}

void GPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret;
  std::vector<std::string> reasons;

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    unsigned int clock = 0;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    unsigned long long clocksThrottleReasons = 0LL;  // NOLINT
    ret = nvmlDeviceGetCurrentClocksThrottleReasons(itr->device, &clocksThrottleReasons);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve current clocks throttling reasons");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    while (clocksThrottleReasons) {
      unsigned long long flag = clocksThrottleReasons & ((~clocksThrottleReasons) + 1);  // NOLINT
      clocksThrottleReasons ^= flag;
      reasons.push_back(reasonToString(flag));

      switch (flag) {
        case nvmlClocksThrottleReasonGpuIdle:
        case nvmlClocksThrottleReasonApplicationsClocksSetting:
        case nvmlClocksThrottleReasonSwPowerCap:
          // we do not treat as error
          break;
        default:
          level = DiagStatus::ERROR;
          break;
      }
    }

    stat.add(fmt::format("GPU {}: status", index), throttling_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

    if (reasons.empty()) {reasons.emplace_back("ReasonNone");}

    stat.add(fmt::format("GPU {}: reasons", index), boost::algorithm::join(reasons, ", "));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, throttling_dict_.at(whole_level));
}

std::string GPUMonitor::toHumanReadable(unsigned long long size)  // NOLINT
{
  const char * units[] = {"B", "K", "M", "G", "T"};
  int count = 0;
  double dsize = size;

  while (dsize > 1024) {
    dsize /= 1024;
    ++count;
  }
  const char * format = (dsize > 0 && dsize < 10) ? "{:.1f}{}" : "{:.0f}{}";
  return fmt::format(format, dsize, units[count]);
}
