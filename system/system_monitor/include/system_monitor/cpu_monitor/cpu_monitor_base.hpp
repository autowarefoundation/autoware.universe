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
 * @file cpu_monitor_base.h
 * @brief CPU monitor base class
 */

#ifndef SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
#define SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <tier4_external_api_msgs/msg/cpu_status.hpp>
#include <tier4_external_api_msgs/msg/cpu_usage.hpp>

#include <climits>
#include <map>
#include <string>
#include <vector>

/**
 * @brief CPU temperature information
 */
typedef struct cpu_temp_info
{
  std::string label_;  //!< @brief cpu label
  std::string path_;   //!< @brief sysfs path to cpu temperature

  cpu_temp_info() : label_(), path_() {}
  cpu_temp_info(const std::string & label, const std::string & path) : label_(label), path_(path) {}
} cpu_temp_info;

/**
 * @brief CPU frequency information
 */
typedef struct cpu_freq_info
{
  int index_;         //!< @brief cpu index
  std::string path_;  //!< @brief sysfs path to cpu frequency

  cpu_freq_info() : index_(0), path_() {}
  cpu_freq_info(int index, const std::string & path) : index_(index), path_(path) {}
} cpu_freq_info;

/**
 * @brief CPU usage information
 */
typedef struct cpu_usage_info
{
  std::string cpu_name_;  //!< @brief cpu name
  int usr_;               //!< @brief usr usage [jiffies]
  int nice_;              //!< @brief nice usage [jiffies]
  int sys_;               //!< @brief sys usage [jiffies]
  int idle_;              //!< @brief idle usage [jiffies]
  int iowait_;            //!< @brief iowait usage [jiffies]
  int irq_;               //!< @brief irq usage [jiffies]
  int soft_;              //!< @brief soft usage [jiffies]
  int steal_;             //!< @brief steal usage [jiffies]

  cpu_usage_info()
  : usr_(-1), nice_(-1), sys_(-1), idle_(-1), iowait_(-1), irq_(-1), soft_(-1), steal_(-1)
  {
  }

  int totalTime() const { return usr_ + nice_ + sys_ + idle_ + iowait_ + irq_ + soft_ + steal_; }

  int totalActiveTime() const { return usr_ + nice_ + sys_ + irq_ + soft_ + steal_; }
} cpu_usage_info;

class CPUMonitorBase : public rclcpp::Node
{
public:
  /**
   * @brief Update the diagnostic state.
   */
  void update();

  /**
   * @brief get names for core temperature files
   */
  virtual void getTempNames();

  /**
   * @brief get names for cpu frequency files
   */
  virtual void getFreqNames();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

  /**
   * @brief constructor
   * @param [in] node_name Name of the node.
   * @param [in] options Options associated with this node.
   */
  CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief check CPU temperature
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkTemp(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief convert Cpu Usage To diagnostic Level
   * @param [cpu_name] mpstat cpu name
   * @param [usage] cpu usage value
   * @return DiagStatus::OK or WARN or ERROR
   */
  virtual int CpuUsageToLevel(const std::string & cpu_name, float usage);

  /**
   * @brief check CPU load average
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkLoad(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU thermal throttling
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  virtual void checkThrottling(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CPU frequency
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  virtual void checkFrequency(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief Timer callback to execute all checks
   */
  virtual void onTimer();

  /**
   * @brief Timeout callback function for executing reading temperature
   */
  virtual void onTempTimeout();

  /**
   * @brief Timeout callback function for executing reading usage
   */
  virtual void onUsageTimeout();

  /**
   * @brief Timeout callback function for executing reading load
   */
  virtual void onLoadTimeout();

  /**
   * @brief Timeout callback function for executing reading frequency
   */
  virtual void onFreqTimeout();

  /**
   * @brief execute reading temperature
   */
  virtual void executeReadTemp();

  /**
   * @brief execute reading usage
   */
  virtual void executeReadUsage();

  /**
   * @brief execute reading load
   */
  virtual void executeReadLoad();

  /**
   * @brief execute reading thermal throttling
   */
  virtual void executeReadThrottling();

  /**
   * @brief execute reading frequency
   */
  virtual void executeReadFrequency();

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  rclcpp::TimerBase::SharedPtr timer_;                     //!< @brief Timer to execute onTempTimer
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;  //!< @brief Callback Group
  rclcpp::TimerBase::SharedPtr timeout_timer_;  //!< @brief Timeout for reading each CPU status

  char hostname_[HOST_NAME_MAX + 1];         //!< @brief host name
  int num_cores_;                            //!< @brief number of cores
  std::vector<cpu_temp_info> temps_;         //!< @brief CPU list for temperature
  std::vector<cpu_freq_info> freqs_;         //!< @brief CPU list for frequency
  std::vector<cpu_usage_info> prev_usages_;  //!< @brief CPU list for usage

  std::vector<int> usage_warn_check_cnt_;   //!< @brief CPU list for usage over warn check counter
  std::vector<int> usage_error_check_cnt_;  //!< @brief CPU list for usage over error check counter
  bool mpstat_exists_;                      //!< @brief flag if mpstat exists

  float usage_warn_;       //!< @brief CPU usage(%) to generate warning
  float usage_error_;      //!< @brief CPU usage(%) to generate error
  int usage_warn_count_;   //!< @brief continuous count over usage_warn_ to generate warning
  int usage_error_count_;  //!< @brief continuous count over usage_error_ to generate error
  bool usage_avg_;         //!< @brief Check CPU usage calculated as averages among all processors

  int temp_timeout_;   //!< @brief Timeout duration for reading temperature
  int usage_timeout_;  //!< @brief Timeout duration for reading usage
  int load_timeout_;   //!< @brief Timeout duration for reading load
  int freq_timeout_;   //!< @brief Timeout duration for reading frequency

  std::mutex temp_mutex_;                  //!< @brief Mutex for output from reading temperature
  std::string temp_error_str_;             //!< @brief Error string
  std::map<std::string, float> temp_map_;  //!< @brief CPU temperature map
  double temp_elapsed_ms_;                 //!< @brief Execution time of reading temperature
  std::mutex temp_timeout_mutex_;  //!< @brief Mutex regarding timeout for reading temperature
  bool temp_timeout_expired_;      //!< @brief Timeout for reading temperature has expired or not

  std::mutex usage_mutex_;                      //!< @brief Mutex for output from reading usage
  std::string usage_error_str_;                 //!< @brief Error string
  std::map<std::string, CpuStatus> usage_map_;  //!< @brief CPU usage map
  double usage_elapsed_ms_;                     //!< @brief Execution time of reading usage
  std::mutex usage_timeout_mutex_;  //!< @brief Mutex regarding timeout for reading usage
  bool usage_timeout_expired_;      //!< @brief Timeout for reading usage has expired or not

  std::mutex load_mutex_;          //!< @brief Mutex for output from reading load
  std::string load_error_str_;     //!< @brief Error string
  double load_avg_[3];             //!< @brief CPU load average
  double load_elapsed_ms_;         //!< @brief Execution time of reading load
  std::mutex load_timeout_mutex_;  //!< @brief Mutex regarding timeout for reading load
  bool load_timeout_expired_;      //!< @brief Timeout for reading load has expired or not

  std::mutex freq_mutex_;          //!< @brief Mutex for output from reading frequency
  std::string freq_error_str_;     //!< @brief Error string
  std::map<int, float> freq_map_;  //!< @brief CPU frequency vector
  double freq_elapsed_ms_;         //!< @brief Execution time of reading frequency
  rclcpp::TimerBase::SharedPtr freq_timeout_timer_;  //!< @brief Timeout for reading frequency
  std::mutex freq_timeout_mutex_;  //!< @brief Mutex regarding timeout for reading frequency
  bool freq_timeout_expired_;      //!< @brief Timeout for reading frequency has expired or not

  /**
   * @brief CPU temperature status messages
   */
  const std::map<int, const char *> temp_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "warm"}, {DiagStatus::ERROR, "hot"}};

  /**
   * @brief CPU usage status messages
   */
  const std::map<int, const char *> load_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}};

  /**
   * @brief CPU thermal throttling status messages
   */
  const std::map<int, const char *> thermal_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "unused"}, {DiagStatus::ERROR, "throttling"}};

  // Publisher
  rclcpp::Publisher<tier4_external_api_msgs::msg::CpuUsage>::SharedPtr pub_cpu_usage_;

  virtual void publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage);
};

#endif  // SYSTEM_MONITOR__CPU_MONITOR__CPU_MONITOR_BASE_HPP_
