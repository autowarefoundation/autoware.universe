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

#include "system_monitor/gpu_monitor/nvml_gpu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/algorithm/string.hpp>

#include <fmt/format.h>
#include <sys/time.h>

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <vector>

GPUMonitor::GPUMonitor(const rclcpp::NodeOptions & options) 
: GPUMonitorBase("gpu_monitor", options),
  temp_timeout_(declare_parameter<int>("temp_timeout", 5)),
  temp_timeout_expired_(false)
  usage_timeout_(declare_parameter<int>("usage_timeout", 5)),
  usage_timeout_expired_(false)
{
  nvmlReturn_t ret = nvmlInit();
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

  for (unsigned int index = 0; index < deviceCount; ++index) {
    gpu_info info{};
    ret = nvmlDeviceGetHandleByIndex(index, &info.device);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to acquire the handle for a particular device [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetName(info.device, info.name, NVML_DEVICE_NAME_BUFFER_SIZE);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to retrieve the name of this device [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    ret = nvmlDeviceGetPciInfo(info.device, &info.pci);
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to retrieve the PCI attributes [%d]: %s", index,
        nvmlErrorString(ret));
      continue;
    }
    if (!getSupportedGPUClocks(index, info.device, info.supported_gpu_clocks)) {
      continue;
    }
    gpus_.push_back(info);
  }
  
  using namespace std::literals::chrono_literals;
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&GPUMonitor::onTimer, this),
    timer_callback_group_);
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
  std::vector<gpu_temp_info> tmp_temp_info_vector;
  double tmp_temp_elapsed_ms = 0.0;

  //thread-sage copy
  {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    tmp_temp_info_vector = temp_info_vector_;
    tmp_temp_elapsed_ms = temp_elapsed_ms_;
  }

  int level = DiagStatus::OK;
  int index = 0;

  if (tmp_temp_info_vector.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = tmp_temp_info_vector.begin(); itr != tmp_temp_info_vector.end(); ++itr, ++index) {
    if (!itr->context.empty()) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current temperature");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci_bus_id);
      stat.add(fmt::format("GPU {}: content", index), itr->context);
      return;
    }

    level = DiagStatus::OK;
    stat.addf(itr->name, "%d.0 DegC", itr->temp);
    if (itr->temp >= temp_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (itr->temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  bool timeout_expired = false;
  {
    std::lock_guard<std::mutex> lock(temp_timeout_mutex_);
    timeout_expired = temp_timeout_expired_;
  }

  if (timeout_expired) {
    stat.summary(DiagStatus::WARN, "Reading Temprature Timeout");
  } else {
    stat.summary(level, temp_dict_.at(level));
  }

  stat.addf("execution time", "%.3f ms", tmp_temp_elapsed_ms);
}

void GPUMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::vector<gpu_usage_info> tmp_usage_info_vector;
  double tmp_usage_elapsed_ms = 0.0;

  //thread-sage copy
  {
    std::lock_guard<std::mutex> lock(usage_mutex_);
    tmp_usage_info_vector = usage_info_vector_;
    tmp_usage_elapsed_ms = usage_elapsed_ms_;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (tmp_usage_info_vector.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = tmp_usage_info_vector.begin(); itr != tmp_usage_info_vector.end(); ++itr, ++index) {
    if (!itr->context.empty()) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current utilization rates");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci_bus_id);
      stat.add(fmt::format("GPU {}: content", index), itr->context);
      return;
    }

    level = DiagStatus::OK;
    float usage = static_cast<float>(itr->usage) / 100.0;
    if (usage >= gpu_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (usage >= gpu_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%d.0%%", itr->usage);

    // Add data to diagnostic
    int add_cnt = 0;
    for (auto itr_util = itr->util_list.begin(); itr_util != itr->util_list.end(); ++itr_util, ++add_cnt) {
      stat.add(fmt::format("GPU {0}: process {1}: pid", index, add_cnt), itr_util->pid);
      stat.add(fmt::format("GPU {0}: process {1}: name", index, add_cnt), itr_util->name);
      stat.addf(
        fmt::format("GPU {0}: process {1}: usage", index, add_cnt), "%ld.0%%", itr_util->smUtil);
      ++add_cnt;
      break;
    }

    whole_level = std::max(whole_level, level);
  }

  bool timeout_expired = false;
  {
    std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
    timeout_expired = usage_timeout_expired_;
  }

  if (timeout_expired) {
    stat.summary(DiagStatus::WARN, "Reading Usage Timeout");
  } else {
    stat.summary(whole_level, load_dict_.at(whole_level));
  }

  stat.addf("execution time", "%.3f ms", tmp_usage_elapsed_ms);
}

// void GPUMonitor::addProcessUsage(
//   int index, nvmlDevice_t device, diagnostic_updater::DiagnosticStatusWrapper & stat)
// {
//   nvmlReturn_t ret{};
//   std::list<uint32_t> running_pid_list;

//   // Get Compute Process ID
//   uint32_t info_count = MAX_ARRAY_SIZE;
//   std::unique_ptr<nvmlProcessInfo_t[]> infos;
//   infos = std::make_unique<nvmlProcessInfo_t[]>(MAX_ARRAY_SIZE);
//   ret = nvmlDeviceGetComputeRunningProcesses(device, &info_count, infos.get());
//   if (ret != NVML_SUCCESS) {
//     RCLCPP_WARN(
//       this->get_logger(), "Failed to nvmlDeviceGetComputeRunningProcesses NVML: %s",
//       nvmlErrorString(ret));
//     return;
//   }
//   for (uint32_t cnt = 0; cnt < info_count; ++cnt) {
//     running_pid_list.push_back(infos[cnt].pid);
//   }

//   // Get Graphics Process ID
//   info_count = MAX_ARRAY_SIZE;
//   infos = std::make_unique<nvmlProcessInfo_t[]>(MAX_ARRAY_SIZE);
//   ret = nvmlDeviceGetGraphicsRunningProcesses(device, &info_count, infos.get());
//   if (ret != NVML_SUCCESS) {
//     RCLCPP_WARN(
//       this->get_logger(), "Failed to nvmlDeviceGetGraphicsRunningProcesses NVML: %s",
//       nvmlErrorString(ret));
//     return;
//   }
//   for (uint32_t cnt = 0; cnt < info_count; ++cnt) {
//     running_pid_list.push_back(infos[cnt].pid);
//   }

//   // Get util_count(1st call of nvmlDeviceGetProcessUtilization)
//   uint32_t util_count = 0;
//   ret = nvmlDeviceGetProcessUtilization(device, NULL, &util_count, current_timestamp_);
//   // This function result will not succeed, because arg[util_count(in)] is 0.
//   if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
//     RCLCPP_WARN(
//       this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(1st) NVML: %s",
//       nvmlErrorString(ret));
//     return;
//   }
//   // Check util_count
//   if (util_count <= 0) {
//     RCLCPP_WARN(this->get_logger(), "Illegal util_count: %d", util_count);
//     return;
//   }

//   // Get utils data(2nd call of nvmlDeviceGetProcessUtilization)
//   std::unique_ptr<nvmlProcessUtilizationSample_t[]> utils;
//   utils = std::make_unique<nvmlProcessUtilizationSample_t[]>(util_count);
//   ret = nvmlDeviceGetProcessUtilization(device, utils.get(), &util_count, current_timestamp_);
//   if (ret != NVML_SUCCESS) {
//     RCLCPP_WARN(
//       this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(2nd) NVML: %s",
//       nvmlErrorString(ret));
//     return;
//   }

//   // Add data to diagnostic
//   int add_cnt = 0;
//   for (uint32_t cnt = 0; cnt < util_count; ++cnt) {
//     for (auto pid : running_pid_list) {
//       // PID check, because it contains illegal PID data. ex) PID:0
//       if (utils[cnt].pid == pid) {
//         char name[MAX_NAME_LENGTH + 1] = {};
//         nvmlSystemGetProcessName(utils[cnt].pid, name, MAX_NAME_LENGTH);
//         stat.add(fmt::format("GPU {0}: process {1}: pid", index, add_cnt), utils[cnt].pid);
//         stat.add(fmt::format("GPU {0}: process {1}: name", index, add_cnt), name);
//         stat.addf(
//           fmt::format("GPU {0}: process {1}: usage", index, add_cnt), "%ld.0%%",
//           ((utils[cnt].smUtil != UINT32_MAX) ? utils[cnt].smUtil : 0));
//         ++add_cnt;
//         break;
//       }
//     }
//   }

//   // Update timestamp(usec)
//   rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
//   current_timestamp_ = system_clock.now().nanoseconds() / 1000;
// }

void GPUMonitor::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

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

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void GPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};
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
      reasons.emplace_back(reasonToString(flag));

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

    if (reasons.empty()) {
      reasons.emplace_back("ReasonNone");
    }

    stat.add(fmt::format("GPU {}: reasons", index), boost::algorithm::join(reasons, ", "));

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, throttling_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
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

void GPUMonitor::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (gpus_.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    int level = DiagStatus::OK;
    unsigned int clock = 0;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
    if (ret != NVML_SUCCESS) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
      stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
      return;
    }

    if (itr->supported_gpu_clocks.find(clock) == itr->supported_gpu_clocks.end()) {
      level = DiagStatus::WARN;
    }

    stat.add(fmt::format("GPU {}: status", index), frequency_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

    whole_level = std::max(whole_level, level);
  }

  stat.summary(whole_level, frequency_dict_.at(whole_level));

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void GPUMonitor::onTimer()
{
  readTemp();
  readUsage();
}

void GPUMonitor::onTempTimeout()
{
  std::lock_guard<std::mutex> lock(temp_timeout_mutex_);
  temp_timeout_expired_ = true;
}

void GPUMonitor::onUsageTimeout()
{
  std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
  usage_timeout_expired_ = true;
}

void GPUMonitor::readTemp()
{
    // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  nvmlReturn_t ret{};
  std::vector<gpu_temp_info> tmp_temp_info_vector;

  // Start timeout timer for read temperature
  {
    std::lock_guard<std::mutex> lock(temp_timeout_mutex_);
    temp_timeout_expired_ = false;
  }
  timeout_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(temp_timeout_), std::bind(&GPUMonitor::onTempTimeout, this));

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr) {
    unsigned int temp = 0;
    gpu_temp_info temp_info;
    ret = nvmlDeviceGetTemperature(itr->device, NVML_TEMPERATURE_GPU, &temp);
    if (ret != NVML_SUCCESS) {
      temp_info.name = itr->name;
      temp_info.pci_bus_id = itr->pci.busId;
      temp_info.context = nvmlErrorString(ret);
    } else {
      temp_info.name = itr->name;
      temp_info.temp = temp;
    }
    tmp_temp_info_vector.push_back(temp_info);
  }

  // Stop timeout timer for read temperature
  timeout_timer_->cancel();

  double elapsed_ms = stop_watch.toc("execution_time");

  //thread-sage copy
  {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    temp_info_vector_ = tmp_temp_info_vector;
    temp_elapsed_ms_ = elapsed_ms;
  }
}

void GPUMonitor::readUsage()
{
    // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::vector<gpu_usage_info> tmp_usage_info_vector;

  // Start timeout timer for read usage
  {
    std::lock_guard<std::mutex> lock(usage_timeout_mutex_);
    usage_timeout_expired_ = false;
  }
  timeout_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(usage_timeout_), std::bind(&GPUMonitor::onUsageTimeout, this));

  int index = 0;
  nvmlReturn_t ret{};

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    gpu_usage_info usage_info;
    ret = nvmlDeviceGetUtilizationRates(itr->device, &itr->utilization);
    if (ret != NVML_SUCCESS) {
      usage_info.name = itr->name;
      usage_info.pci_bus_id = itr->pci.busId;
      usage_info.context = nvmlErrorString(ret);
    } else {
      usage_info.name = itr->name;
      usage_info.usage = itr->utilization.gpu;
      addProcessUsage(itr->device, usage_info.util_list);
    }
    tmp_usage_info_vector.push_back(usage_info);
  }

  // Stop timeout timer for read usage
  timeout_timer_->cancel();

  double elapsed_ms = stop_watch.toc("execution_time");

  //thread-sage copy
  {
    std::lock_guard<std::mutex> lock(usage_mutex_);
    usage_info_vector_ = tmp_usage_info_vector;
    usage_elapsed_ms_ = elapsed_ms;
  }
}

void GPUMonitor::addProcessUsage(nvmlDevice_t device, std::list<gpu_util_info> & util_list)
{
  nvmlReturn_t ret{};
  std::list<uint32_t> running_pid_list;

  // Get Compute Process ID
  uint32_t info_count = MAX_ARRAY_SIZE;
  std::unique_ptr<nvmlProcessInfo_t[]> infos;
  infos = std::make_unique<nvmlProcessInfo_t[]>(MAX_ARRAY_SIZE);
  ret = nvmlDeviceGetComputeRunningProcesses(device, &info_count, infos.get());
  if (ret != NVML_SUCCESS) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to nvmlDeviceGetComputeRunningProcesses NVML: %s",
      nvmlErrorString(ret));
    return;
  }
  for (uint32_t cnt = 0; cnt < info_count; ++cnt) {
    running_pid_list.push_back(infos[cnt].pid);
  }

  // Get Graphics Process ID
  info_count = MAX_ARRAY_SIZE;
  infos = std::make_unique<nvmlProcessInfo_t[]>(MAX_ARRAY_SIZE);
  ret = nvmlDeviceGetGraphicsRunningProcesses(device, &info_count, infos.get());
  if (ret != NVML_SUCCESS) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to nvmlDeviceGetGraphicsRunningProcesses NVML: %s",
      nvmlErrorString(ret));
    return;
  }
  for (uint32_t cnt = 0; cnt < info_count; ++cnt) {
    running_pid_list.push_back(infos[cnt].pid);
  }

  // Get util_count(1st call of nvmlDeviceGetProcessUtilization)
  uint32_t util_count = 0;
  ret = nvmlDeviceGetProcessUtilization(device, NULL, &util_count, current_timestamp_);
  // This function result will not succeed, because arg[util_count(in)] is 0.
  if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(1st) NVML: %s",
      nvmlErrorString(ret));
    return;
  }
  // Check util_count
  if (util_count <= 0) {
    RCLCPP_WARN(this->get_logger(), "Illegal util_count: %d", util_count);
    return;
  }

  // Get utils data(2nd call of nvmlDeviceGetProcessUtilization)
  std::unique_ptr<nvmlProcessUtilizationSample_t[]> utils;
  utils = std::make_unique<nvmlProcessUtilizationSample_t[]>(util_count);
  ret = nvmlDeviceGetProcessUtilization(device, utils.get(), &util_count, current_timestamp_);
  if (ret != NVML_SUCCESS) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to nvmlDeviceGetProcessUtilization(2nd) NVML: %s",
      nvmlErrorString(ret));
    return;
  }

  for (uint32_t cnt = 0; cnt < util_count; ++cnt) {
    for (auto pid : running_pid_list) {
      // PID check, because it contains illegal PID data. ex) PID:0
      if (utils[cnt].pid == pid) {
        char name[MAX_NAME_LENGTH + 1] = {};
        gpu_util_info util;
        nvmlSystemGetProcessName(utils[cnt].pid, name, MAX_NAME_LENGTH);
        util.pid = utils[cnt].pid;
        util.name = std::string(name);
        util.smUtil = (utils[cnt].smUtil != UINT32_MAX) ? utils[cnt].smUtil : 0;
        util_list.push_back(util);
        break;
      }
    }
  }

  // Update timestamp(usec)
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  current_timestamp_ = system_clock.now().nanoseconds() / 1000;
}

void GPUMonitor::readMemoryUsage()
{
  // Start to measure elapsed time


  int index = 0;
  nvmlReturn_t ret{};

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

// void GPUMonitor::checkThrottling()
// {
//   // Remember start time to measure elapsed time
//   const auto t_start = SystemMonitorUtility::startMeasurement();

//   int level = DiagStatus::OK;
//   int whole_level = DiagStatus::OK;
//   int index = 0;
//   nvmlReturn_t ret{};
//   std::vector<std::string> reasons;

//   if (gpus_.empty()) {
//     stat.summary(DiagStatus::ERROR, "gpu not found");
//     return;
//   }

//   for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
//     unsigned int clock = 0;
//     ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
//     if (ret != NVML_SUCCESS) {
//       stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
//       stat.add(fmt::format("GPU {}: name", index), itr->name);
//       stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
//       stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
//       return;
//     }

//     unsigned long long clocksThrottleReasons = 0LL;  // NOLINT
//     ret = nvmlDeviceGetCurrentClocksThrottleReasons(itr->device, &clocksThrottleReasons);
//     if (ret != NVML_SUCCESS) {
//       stat.summary(DiagStatus::ERROR, "Failed to retrieve current clocks throttling reasons");
//       stat.add(fmt::format("GPU {}: name", index), itr->name);
//       stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
//       stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
//       return;
//     }

//     while (clocksThrottleReasons) {
//       unsigned long long flag = clocksThrottleReasons & ((~clocksThrottleReasons) + 1);  // NOLINT
//       clocksThrottleReasons ^= flag;
//       reasons.emplace_back(reasonToString(flag));

//       switch (flag) {
//         case nvmlClocksThrottleReasonGpuIdle:
//         case nvmlClocksThrottleReasonApplicationsClocksSetting:
//         case nvmlClocksThrottleReasonSwPowerCap:
//           // we do not treat as error
//           break;
//         default:
//           level = DiagStatus::ERROR;
//           break;
//       }
//     }

//     stat.add(fmt::format("GPU {}: status", index), throttling_dict_.at(level));
//     stat.add(fmt::format("GPU {}: name", index), itr->name);
//     stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

//     if (reasons.empty()) {
//       reasons.emplace_back("ReasonNone");
//     }

//     stat.add(fmt::format("GPU {}: reasons", index), boost::algorithm::join(reasons, ", "));

//     whole_level = std::max(whole_level, level);
//   }

//   stat.summary(whole_level, throttling_dict_.at(whole_level));

//   // Measure elapsed time since start time and report
//   SystemMonitorUtility::stopMeasurement(t_start, stat);
// }

// void GPUMonitor::checkFrequency()
// {
//   // Remember start time to measure elapsed time
//   const auto t_start = SystemMonitorUtility::startMeasurement();

//   int whole_level = DiagStatus::OK;
//   int index = 0;
//   nvmlReturn_t ret{};

//   if (gpus_.empty()) {
//     stat.summary(DiagStatus::ERROR, "gpu not found");
//     return;
//   }

//   for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
//     int level = DiagStatus::OK;
//     unsigned int clock = 0;
//     ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &clock);
//     if (ret != NVML_SUCCESS) {
//       stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
//       stat.add(fmt::format("GPU {}: name", index), itr->name);
//       stat.add(fmt::format("GPU {}: bus-id", index), itr->pci.busId);
//       stat.add(fmt::format("GPU {}: content", index), nvmlErrorString(ret));
//       return;
//     }

//     if (itr->supported_gpu_clocks.find(clock) == itr->supported_gpu_clocks.end()) {
//       level = DiagStatus::WARN;
//     }

//     stat.add(fmt::format("GPU {}: status", index), frequency_dict_.at(level));
//     stat.add(fmt::format("GPU {}: name", index), itr->name);
//     stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", clock);

//     whole_level = std::max(whole_level, level);
//   }

//   stat.summary(whole_level, frequency_dict_.at(whole_level));

//   // Measure elapsed time since start time and report
//   SystemMonitorUtility::stopMeasurement(t_start, stat);
// }

bool GPUMonitor::getSupportedGPUClocks(
  int index, nvmlDevice_t & device, std::set<unsigned int> & supported_gpu_clocks)
{
  unsigned int mem_clock_count = 0;
  nvmlReturn_t ret{};

  ret = nvmlDeviceGetSupportedMemoryClocks(device, &mem_clock_count, nullptr);
  if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the count of possible memory clocks [%d]: %s", index,
      nvmlErrorString(ret));
    return false;
  }

  std::shared_ptr<unsigned int[]> mem_clocks(new unsigned int[mem_clock_count]);
  ret = nvmlDeviceGetSupportedMemoryClocks(device, &mem_clock_count, mem_clocks.get());
  if (ret != NVML_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to retrieve the list of possible memory clocks [%d]: %s", index,
      nvmlErrorString(ret));
    return false;
  }

  for (unsigned int mem_clock_index = 0; mem_clock_index < mem_clock_count; mem_clock_index++) {
    unsigned int gpu_clock_count = 0;

    ret = nvmlDeviceGetSupportedGraphicsClocks(
      device, mem_clocks[mem_clock_index], &gpu_clock_count, nullptr);
    if (ret != NVML_ERROR_INSUFFICIENT_SIZE) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the count of possible graphics clocks for %d MHz memory clock [%d]: %s",
        mem_clocks[mem_clock_index], index, nvmlErrorString(ret));
      return false;
    }

    std::shared_ptr<unsigned int[]> gpu_clocks(new unsigned int[gpu_clock_count]);
    ret = nvmlDeviceGetSupportedGraphicsClocks(
      device, mem_clocks[mem_clock_index], &gpu_clock_count, gpu_clocks.get());
    if (ret != NVML_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to retrieve the list of possible graphics clocks for %d MHz memory clock [%d]: %s",
        mem_clocks[mem_clock_index], index, nvmlErrorString(ret));
      return false;
    }
    for (unsigned int gpu_clock_index = 0; gpu_clock_index < gpu_clock_count; gpu_clock_index++) {
      supported_gpu_clocks.insert(gpu_clocks[gpu_clock_index]);
    }
  }
  return true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GPUMonitor)
