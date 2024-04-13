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
 * @file nvml_gpu_monitor.h
 * @brief NVML GPU monitor class
 */

#ifndef SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_
#define SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_

#include "system_monitor/gpu_monitor/gpu_monitor_base.hpp"

#include <nvml.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#define reasonToString(X)                                                                  \
  (((X)&nvmlClocksThrottleReasonGpuIdle)                     ? "GpuIdle"                   \
   : ((X)&nvmlClocksThrottleReasonApplicationsClocksSetting) ? "ApplicationsClocksSetting" \
   : ((X)&nvmlClocksThrottleReasonSwPowerCap)                ? "SwPowerCap"                \
   : ((X)&nvmlClocksThrottleReasonHwSlowdown)                ? "HwSlowdown"                \
   : ((X)&nvmlClocksThrottleReasonSyncBoost)                 ? "SyncBoost"                 \
   : ((X)&nvmlClocksThrottleReasonSwThermalSlowdown)         ? "SwThermalSlowdown"         \
   : ((X)&nvmlClocksThrottleReasonHwThermalSlowdown)         ? "HwThermalSlowdown"         \
   : ((X)&nvmlClocksThrottleReasonHwPowerBrakeSlowdown)      ? "HwPowerBrakeSlowdown"      \
   : ((X)&nvmlClocksThrottleReasonDisplayClockSetting)       ? "DisplayClockSetting"       \
                                                             : "UNKNOWN")

/**
 * @brief GPU information
 */
struct gpu_info
{
  nvmlDevice_t device;                          //!< @brief handle for a particular device
  char name[NVML_DEVICE_NAME_BUFFER_SIZE];      //!< @brief name of device
  nvmlPciInfo_t pci;                            //!< @brief PCI information about a GPU device
  nvmlUtilization_t utilization;                //!< @brief Utilization information for a device
  std::set<unsigned int> supported_gpu_clocks;  //!< @brief list of supported GPU clocks
};

/**
 * @brief GPU temperature information
 */
struct gpu_temp_info
{
  std::string name;  //!< @brief name of device
  unsigned int temp;    //!< @brief temperature of device
  std::string pci_bus_id;  //!< @brief PCI bus ID of device
  std::string context;  //!< @brief error message
};

/**
 * @brief GPU util information
 */
struct gpu_util_info
{
  unsigned int pid;  //!< @brief process ID
  std::string name;  //!< @brief process name
  unsigned int smUtil;  //!< @brief SM utilization
};

/**
 * @brief GPU usage information
 */
struct gpu_usage_info
{
  std::string name;  //!< @brief name of device
  float usage;    //!< @brief usage of device
  std::string pci_bus_id;  //!< @brief PCI bus ID of device
  std::string context;  //!< @brief error message
  std::list<gpu_util_info> util_list;  //!< @brief list of process usage
}; 

/**
 * @brief GPU temperature information
 */
struct gpu_memory_usage_info
{
  std::string name;  //!< @brief name of device
  unsigned int memory_usage;    //!< @brief temperature of device
  std::string pci_bus_id;  //!< @brief PCI bus ID of device
  std::string context;  //!< @brief error message
  nvmlMemory_t memory_detail; //!< @brief memory detail
};

/**
 * @brief GPU throttling information
 */
struct gpu_throttling_info
{
  std::string name;  //!< @brief name of device
  std::vector<std::string> reasons;    //!< @brief reason of throttling
  unsigned int clock;    //!< @brief clock of device
  std::string pci_bus_id;  //!< @brief PCI bus ID of device
  std::string context;  //!< @brief error message
  std::string summary;  //!< @brief summary of throttling
  int level;  //!< @brief level of throttling
  gpu_throttling_info() : clock(0) {}
};

/**
 * @brief GPU frequency information
 */
struct gpu_frequency_info
{
  std::string name;  //!< @brief name of device
  unsigned int clock;    //!< @brief clock of device
  std::string pci_bus_id;  //!< @brief PCI bus ID of device
  std::string context;  //!< @brief error message
  int level;  //!< @brief level of throttling
  gpu_frequency_info() : clock(0) {}
};


class GPUMonitor : public GPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  explicit GPUMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Terminate the node, log final statements. An independent function is preferred to allow
   * an explicit way to operate actions that require a valid rclcpp context. By default this method
   * does nothing.
   */
  void shut_down() override;

protected:
  /**
   * @brief check GPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkTemp(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief add stat of GPU usage per process
   * @param [in] index GPU index
   * @param [in] device GPU device
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void addProcessUsage(
    nvmlDevice_t device, std::list<gpu_util_info> & util_list);

  /**
   * @brief check GPU memory usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkMemoryUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   *  
   * @brief check GPU information
   */
  void onTimer();

  /**
   * @brief timeout timer for temperature
   */
  void onTempTimeout();

  /**
   * @brief timeout timer for usage
   */
  void onUsageTimeout();

  /**
   * @brief timeout timer for memory usage
   */
  void onMemoryUsageTimeout();

  /**
   * @brief timeout timer for throttling
   */
  void onThrottlingTimeout();

  /**
   * @brief timeout timer for frequency
   */
  void onFrequencyTimeout();


  /**
   * @brief read GPU temperature
   */
  void readTemp();

  /**
   * @brief read GPU usage
   */
  void readUsage();

  /**
   * @brief read GPU memory usage
   */
  void readMemoryUsage();

  /**
   * @brief read GPU throttling
   */
  void readThrottling();

  /**
   * @brief read GPU frequency
   */
  void readFrequency();


  /**
   * @brief get human-readable output for memory size
   * @param [in] size size with bytes
   * @return human-readable output
   * @note NOLINT syntax is needed since struct nvmlMemory_t has unsigned long long values to return
   * memory size.
   */
  std::string toHumanReadable(unsigned long long size);  // NOLINT(runtime/int)

  /**
   * @brief check GPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief get supported GPU clocks
   * @param [in] index GPU index
   * @param [in] device GPU device
   * @param [out] list of supported GPU clocks
   * @return result of getting supported GPU clocks
   */
  bool getSupportedGPUClocks(
    int index, nvmlDevice_t & device, std::set<unsigned int> & supported_gpu_clocks);

  static const size_t MAX_ARRAY_SIZE = 64;
  static const size_t MAX_NAME_LENGTH = 128;

  rclcpp::TimerBase::SharedPtr timer_;  //!< @brief timer for monitoring
  rclcpp::TimerBase::SharedPtr timeout_timer_;  //!< @brief timer for temperature timeout
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;  //!< @brief callback group for timer

  std::vector<gpu_info> gpus_;      //!< @brief list of gpus
  uint64_t current_timestamp_ = 0;  //!< @brief latest timestamp[usec] of addProcessUsage()

  std::mutex temp_mutex_; //!< @brief mutex for temperature
  std::vector<gpu_temp_info> temp_info_vector_;  //!< @brief list of temperature information
  int temp_timeout_;                   //!< @brief timeout for temperature
  double temp_elapsed_ms_;              //!< @brief elapsed time for temperature
  std::mutex temp_timeout_mutex_;         //!< @brief mutex for temperature timeout
  bool temp_timeout_expired_;             //!< @brief timeout for temperature has expired or not

  std::mutex usage_mutex_;  //!< @brief mutex for usage
  std::vector<gpu_usage_info> usage_info_vector_;  //!< @brief list of usage information
  int usage_timeout_;                   //!< @brief timeout for usage
  double usage_elapsed_ms_;              //!< @brief elapsed time for usage
  std::mutex usage_timeout_mutex_;         //!< @brief mutex for usage timeout
  bool usage_timeout_expired_;             //!< @brief timeout for usage has expired or not

  std::mutex memory_usage_mutex_;  //!< @brief mutex for memory usage
  std::vector<gpu_memory_usage_info> memory_usage_info_vector_;  //!< @brief list of memory usage information
  int memory_usage_timeout_;                   //!< @brief timeout for memory usage
  double memory_usage_elapsed_ms_;              //!< @brief elapsed time for memory usage
  std::mutex memory_usage_timeout_mutex_;         //!< @brief mutex for memory usage timeout
  bool memory_usage_timeout_expired_;             //!< @brief timeout for memory usage has expired or not

  std::mutex throttling_mutex_;  //!< @brief mutex for throttling
  std::vector<gpu_throttling_info> throttling_info_vector_;  //!< @brief list of throttling information
  int throttling_timeout_;                   //!< @brief timeout for throttling
  double throttling_elapsed_ms_;              //!< @brief elapsed time for throttling
  std::mutex throttling_timeout_mutex_;         //!< @brief mutex for throttling timeout
  bool throttling_timeout_expired_;             //!< @brief timeout for throttling has expired or not

  std::mutex frequency_mutex_;  //!< @brief mutex for frequency
  std::vector<gpu_frequency_info> frequency_info_vector_;  //!< @brief list of frequency information
  int frequency_timeout_;                   //!< @brief timeout for frequency
  double frequency_elapsed_ms_;              //!< @brief elapsed time for frequency
  std::mutex frequency_timeout_mutex_;         //!< @brief mutex for frequency timeout
  bool frequency_timeout_expired_;             //!< @brief timeout for frequency has expired or not

  /**
   * @brief GPU frequency status messages
   */
  const std::map<int, const char *> frequency_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unsupported clock"}};
};

#endif  // SYSTEM_MONITOR__GPU_MONITOR__NVML_GPU_MONITOR_HPP_
