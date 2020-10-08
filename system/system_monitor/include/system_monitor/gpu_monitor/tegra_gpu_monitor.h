#ifndef SYSTEM_MONITOR_GPU_MONITOR_TEGRA_GPU_MONITOR_H
#define SYSTEM_MONITOR_GPU_MONITOR_TEGRA_GPU_MONITOR_H
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
 * @file tegra_gpu_monitor.h
 * @brief Tegra TGPU monitor class
 */

#include <system_monitor/gpu_monitor/gpu_monitor_base.h>
#include <string>
#include <vector>

typedef struct gpu_info
{
  std::string label_;  //!< @brief gpu label
  std::string path_;   //!< @brief sysfs path to gpu temperature

  gpu_info() : label_(), path_() {}
  gpu_info(const std::string & l, const std::string & p) : label_(l), path_(p) {}
} gpu_info;

class GPUMonitor : public GPUMonitorBase
{
public:
  /**
   * @brief constructor
   * @param [in] nh node handle to access global parameters
   * @param [in] pnh node handle to access private parameters
   */
  GPUMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);

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
   * @brief check GPU throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief check GPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat) override;  // NOLINT(runtime/references)

  /**
   * @brief get names for gpu temperature files
   */
  void getTempNames(void);

  /**
   * @brief get names for gpu load files
   */
  void getLoadNames(void);

  /**
   * @brief get names for gpu frequency files
   */
  void getFreqNames(void);

  std::vector<gpu_info> temps_;  //!< @brief GPU list for temperature
  std::vector<gpu_info> loads_;  //!< @brief GPU list for utilization
  std::vector<gpu_info> freqs_;  //!< @brief GPU list for frequency
};

#endif  // SYSTEM_MONITOR_GPU_MONITOR_TEGRA_GPU_MONITOR_H
