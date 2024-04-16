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
  temp_elapsed_ms_(0.0),
  usage_timeout_(declare_parameter<int>("usage_timeout", 5)),
  usage_elapsed_ms_(0.0),
  memory_usage_timeout_(declare_parameter<int>("memory_usage_timeout", 5)),
  memory_usage_elapsed_ms_(0.0),
  throttling_timeout_(declare_parameter<int>("throttling_timeout", 5)),
  throttling_elapsed_ms_(0.0),
  frequency_timeout_(declare_parameter<int>("frequency_timeout", 5)),
  frequency_elapsed_ms_(0.0)
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
    this, get_clock(), 1s, std::bind(&GPUMonitor::onTimer, this), timer_callback_group_);
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

  // thread-sage copy
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

  if (level == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, temp_dict_.at(level));
  } else if (tmp_temp_elapsed_ms == 0.0) {
    stat.summary(DiagStatus::WARN, "Reading Temprature ERROR");
  } else if (tmp_temp_elapsed_ms > temp_timeout_) {
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

  // thread-sage copy
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

  for (auto itr = tmp_usage_info_vector.begin(); itr != tmp_usage_info_vector.end();
       ++itr, ++index) {
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
    for (auto itr_util = itr->util_list.begin(); itr_util != itr->util_list.end();
         ++itr_util, ++add_cnt) {
      stat.add(fmt::format("GPU {0}: process {1}: pid", index, add_cnt), itr_util->pid);
      stat.add(fmt::format("GPU {0}: process {1}: name", index, add_cnt), itr_util->name);
      stat.addf(
        fmt::format("GPU {0}: process {1}: usage", index, add_cnt), "%ld.0%%", itr_util->smUtil);
      ++add_cnt;
      break;
    }

    whole_level = std::max(whole_level, level);
  }

  if (whole_level == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, load_dict_.at(whole_level));
  } else if (tmp_usage_elapsed_ms == 0.0) {
    stat.summary(DiagStatus::WARN, "Reading Usage ERROR");
  } else if (tmp_usage_elapsed_ms > usage_timeout_) {
    stat.summary(DiagStatus::WARN, "Reading Usage Timeout");
  } else {
    stat.summary(whole_level, load_dict_.at(whole_level));
  }

  stat.addf("execution time", "%.3f ms", tmp_usage_elapsed_ms);
}

void GPUMonitor::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::vector<gpu_memory_usage_info> tmp_memory_usage_info_vector;
  double tmp_memory_usage_elapsed_ms = 0.0;

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(memory_usage_mutex_);
    tmp_memory_usage_info_vector = memory_usage_info_vector_;
    tmp_memory_usage_elapsed_ms = memory_usage_elapsed_ms_;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;

  if (tmp_memory_usage_info_vector.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = tmp_memory_usage_info_vector.begin(); itr != tmp_memory_usage_info_vector.end();
       ++itr, ++index) {
    if (!itr->context.empty()) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current memory usage");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci_bus_id);
      stat.add(fmt::format("GPU {}: content", index), itr->context);
      return;
    }

    level = DiagStatus::OK;
    float usage = static_cast<float>(itr->memory_usage) / 100.0;
    if (usage >= memory_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (usage >= memory_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }

    stat.add(fmt::format("GPU {}: status", index), load_dict_.at(level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: usage", index), "%d.0%%", itr->memory_usage);
    stat.add(fmt::format("GPU {}: total", index), toHumanReadable(itr->memory_detail.total));
    stat.add(fmt::format("GPU {}: used", index), toHumanReadable(itr->memory_detail.used));
    stat.add(fmt::format("GPU {}: free", index), toHumanReadable(itr->memory_detail.free));

    whole_level = std::max(whole_level, level);
  }

  if (whole_level == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, load_dict_.at(whole_level));
  } else if (tmp_memory_usage_elapsed_ms == 0.0) {
    stat.summary(DiagStatus::WARN, "Reading Memory Usage ERROR");
  } else if (tmp_memory_usage_elapsed_ms > memory_usage_timeout_) {
    stat.summary(DiagStatus::WARN, "Reading Memory Usage Timeout");
  } else {
    stat.summary(whole_level, load_dict_.at(whole_level));
  }

  stat.addf("execution time", "%.3f ms", tmp_memory_usage_elapsed_ms);
}

void GPUMonitor::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::vector<gpu_throttling_info> tmp_throttling_info_vector;
  double tmp_throttling_elapsed_ms = 0.0;

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(throttling_mutex_);
    tmp_throttling_info_vector = throttling_info_vector_;
    tmp_throttling_elapsed_ms = throttling_elapsed_ms_;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (tmp_throttling_info_vector.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = tmp_throttling_info_vector.begin(); itr != tmp_throttling_info_vector.end();
       ++itr, ++index) {
    if (!itr->context.empty()) {
      stat.summary(DiagStatus::ERROR, itr->summary);
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci_bus_id);
      stat.add(fmt::format("GPU {}: content", index), itr->context);
      return;
    }

    stat.add(fmt::format("GPU {}: status", index), throttling_dict_.at(itr->level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", itr->clock);

    if (itr->reasons.empty()) {
      itr->reasons.emplace_back("ReasonNone");
    }

    stat.add(fmt::format("GPU {}: reasons", index), boost::algorithm::join(itr->reasons, ", "));

    whole_level = std::max(whole_level, itr->level);
  }

  if (whole_level == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, throttling_dict_.at(whole_level));
  } else if (tmp_throttling_elapsed_ms == 0.0) {
    stat.summary(DiagStatus::WARN, "Reading Throttling ERROR");
  } else if (tmp_throttling_elapsed_ms > throttling_timeout_) {
    stat.summary(DiagStatus::WARN, "Reading Throttling Timeout");
  } else {
    stat.summary(whole_level, throttling_dict_.at(whole_level));
  }

  stat.addf("execution time", "%.3f ms", tmp_throttling_elapsed_ms);
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
  std::vector<gpu_frequency_info> tmp_frequency_info_vector;
  double tmp_frequency_elapsed_ms = 0.0;

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    tmp_frequency_info_vector = frequency_info_vector_;
    tmp_frequency_elapsed_ms = frequency_elapsed_ms_;
  }

  int whole_level = DiagStatus::OK;
  int index = 0;
  nvmlReturn_t ret{};

  if (tmp_frequency_info_vector.empty()) {
    stat.summary(DiagStatus::ERROR, "gpu not found");
    return;
  }

  for (auto itr = tmp_frequency_info_vector.begin(); itr != tmp_frequency_info_vector.end();
       ++itr, ++index) {
    if (!itr->context.empty()) {
      stat.summary(DiagStatus::ERROR, "Failed to retrieve the current clock speeds");
      stat.add(fmt::format("GPU {}: name", index), itr->name);
      stat.add(fmt::format("GPU {}: bus-id", index), itr->pci_bus_id);
      stat.add(fmt::format("GPU {}: content", index), itr->context);
      return;
    }

    stat.add(fmt::format("GPU {}: status", index), frequency_dict_.at(itr->level));
    stat.add(fmt::format("GPU {}: name", index), itr->name);
    stat.addf(fmt::format("GPU {}: graphics clock", index), "%d MHz", itr->clock);
    whole_level = std::max(whole_level, itr->level);
  }

  if (whole_level == DiagStatus::ERROR) {
    stat.summary(DiagStatus::ERROR, frequency_dict_.at(whole_level));
  } else if (tmp_frequency_elapsed_ms == 0.0) {
    stat.summary(DiagStatus::WARN, "Reading Frequency ERROR");
  } else if (tmp_frequency_elapsed_ms > frequency_timeout_) {
    stat.summary(DiagStatus::WARN, "Reading Frequency Timeout");
  } else {
    stat.summary(whole_level, frequency_dict_.at(whole_level));
  }

  stat.addf("execution time", "%.3f ms", tmp_frequency_elapsed_ms);
}

void GPUMonitor::onTimer()
{
  readTemp();
  readUsage();
  readMemoryUsage();
  readThrottling();
  readFrequency();
}

void GPUMonitor::readTemp()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  nvmlReturn_t ret{};
  std::vector<gpu_temp_info> tmp_temp_info_vector;

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

  double elapsed_ms = stop_watch.toc("execution_time");

  // thread-sage copy
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

  int index = 0;
  nvmlReturn_t ret{};

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr, ++index) {
    gpu_usage_info usage_info;
    ret = nvmlDeviceGetUtilizationRates(itr->device, &itr->utilization);
    if (ret != NVML_SUCCESS) {
      usage_info.name = itr->name;
      usage_info.pci_bus_id = itr->pci.busId;
      usage_info.context = nvmlErrorString(ret);
      tmp_usage_info_vector.push_back(usage_info);
      break;
    } else {
      usage_info.name = itr->name;
      usage_info.usage = itr->utilization.gpu;
      addProcessUsage(itr->device, usage_info.util_list);
      tmp_usage_info_vector.push_back(usage_info);
    }
  }

  double elapsed_ms = stop_watch.toc("execution_time");

  // thread-sage copy
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
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::vector<gpu_memory_usage_info> tmp_memory_usage_info_vector;

  nvmlReturn_t ret{};

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr) {
    nvmlMemory_t memory;
    gpu_memory_usage_info memory_usage_info;
    ret = nvmlDeviceGetMemoryInfo(itr->device, &memory);
    if (ret != NVML_SUCCESS) {
      memory_usage_info.name = itr->name;
      memory_usage_info.pci_bus_id = itr->pci.busId;
      memory_usage_info.context = nvmlErrorString(ret);
      tmp_memory_usage_info_vector.push_back(memory_usage_info);
      break;
    } else {
      memory_usage_info.name = itr->name;
      memory_usage_info.memory_usage = itr->utilization.memory;
      memory_usage_info.memory_detail = memory;
      tmp_memory_usage_info_vector.push_back(memory_usage_info);
    }
  }

  double elapsed_ms = stop_watch.toc("execution_time");

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(memory_usage_mutex_);
    memory_usage_info_vector_ = tmp_memory_usage_info_vector;
    memory_usage_elapsed_ms_ = elapsed_ms;
  }
}

void GPUMonitor::readThrottling()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::vector<gpu_throttling_info> tmp_throttling_info_vector;

  nvmlReturn_t ret{};

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr) {
    gpu_throttling_info throttling_info;
    throttling_info.level = DiagStatus::OK;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &throttling_info.clock);
    if (ret != NVML_SUCCESS) {
      throttling_info.summary = "Failed to retrieve the current clock speeds";
      throttling_info.name = itr->name;
      throttling_info.pci_bus_id = itr->pci.busId;
      throttling_info.context = nvmlErrorString(ret);
      tmp_throttling_info_vector.push_back(throttling_info);
      break;
    }

    unsigned long long clocksThrottleReasons = 0LL;  // NOLINT
    ret = nvmlDeviceGetCurrentClocksThrottleReasons(itr->device, &clocksThrottleReasons);
    if (ret != NVML_SUCCESS) {
      throttling_info.summary = "Failed to retrieve current clocks throttling reasons";
      throttling_info.name = itr->name;
      throttling_info.pci_bus_id = itr->pci.busId;
      throttling_info.context = nvmlErrorString(ret);
      tmp_throttling_info_vector.push_back(throttling_info);
      break;
    }

    while (clocksThrottleReasons) {
      unsigned long long flag = clocksThrottleReasons & ((~clocksThrottleReasons) + 1);  // NOLINT
      clocksThrottleReasons ^= flag;
      throttling_info.reasons.emplace_back(reasonToString(flag));

      switch (flag) {
        case nvmlClocksThrottleReasonGpuIdle:
        case nvmlClocksThrottleReasonApplicationsClocksSetting:
        case nvmlClocksThrottleReasonSwPowerCap:
          // we do not treat as error
          break;
        default:
          throttling_info.level = DiagStatus::ERROR;
          break;
      }
    }

    if (throttling_info.reasons.empty()) {
      throttling_info.reasons.emplace_back("ReasonNone");
    }

    tmp_throttling_info_vector.push_back(throttling_info);
  }

  double elapsed_ms = stop_watch.toc("execution_time");

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(throttling_mutex_);
    throttling_info_vector_ = tmp_throttling_info_vector;
    throttling_elapsed_ms_ = elapsed_ms;
  }
}

void GPUMonitor::readFrequency()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::vector<gpu_frequency_info> tmp_frequency_info_vector;

  nvmlReturn_t ret{};

  for (auto itr = gpus_.begin(); itr != gpus_.end(); ++itr) {
    gpu_frequency_info frequency_info;
    frequency_info.level = DiagStatus::OK;
    ret = nvmlDeviceGetClockInfo(itr->device, NVML_CLOCK_GRAPHICS, &frequency_info.clock);
    if (ret != NVML_SUCCESS) {
      frequency_info.name = itr->name;
      frequency_info.pci_bus_id = itr->pci.busId;
      frequency_info.context = nvmlErrorString(ret);
      tmp_frequency_info_vector.push_back(frequency_info);
      break;
    }

    frequency_info.name = itr->name;

    if (itr->supported_gpu_clocks.find(frequency_info.clock) == itr->supported_gpu_clocks.end()) {
      frequency_info.level = DiagStatus::WARN;
    }

    tmp_frequency_info_vector.push_back(frequency_info);
  }

  double elapsed_ms = stop_watch.toc("execution_time");

  // thread-sage copy
  {
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    frequency_info_vector_ = tmp_frequency_info_vector;
    frequency_elapsed_ms_ = elapsed_ms;
  }
}

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
