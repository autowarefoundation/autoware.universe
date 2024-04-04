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
 * @file mem_monitor.h
 * @brief Memory monitor class
 */

#ifndef SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_
#define SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <boost/process.hpp>

#include <climits>
#include <map>
#include <string>

namespace bp = boost::process;

class MemMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit MemMonitor(const rclcpp::NodeOptions & options);

  /**
   * @brief Update the diagnostic state.
   */
  void update();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check Memory usage
   * @param @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check Memory ECC
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void checkEcc(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief read /proc/meminfo
   */
  void readMemInfo(std::unordered_map<std::string, size_t> & memInfo);

  /**
   * @brief call readMemInfo and calculate memory usage
   */
  std::string readUsage(std::map<std::string, size_t> & map);

  /**
   * @brief execute edac-util command
   */
  std::string executeEdacUtil(std::string & output, std::string & pipe2_error_str);

  /**
   * @brief Timer callback to execute read infomation about usages and ecc
   */
  void onTimer();

  /**
   * @brief Timeout callback function for executing checkUsage
   */
  void onUsageTimeout();

  /**
   * @brief Timeout callback function for executing checkEcc
   */
  void onEccTimeout();

  /**
   * @brief get human-readable output for memory size
   * @param [in] str size with bytes
   * @return human-readable output
   */
  std::string toHumanReadable(const std::string & str);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics

  char hostname_[HOST_NAME_MAX + 1];  //!< @brief host name

  size_t available_size_;  //!< @brief Memory available size to generate error
  int usage_timeout_;      //!< @brief Timeout duration for executing readUsage
  int ecc_timeout_;        //!< @brief Timeout duration for executing edac-util command

  rclcpp::TimerBase::SharedPtr
    timer_;  //!< @brief Timer to execute readUsage and edac-utils command
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;  //!< @brief Callback Group

  rclcpp::TimerBase::SharedPtr timeout_timer_;  //!< @brief Timer for executing readUsage
  std::mutex usage_mutex_;                      //!< @brief Mutex for output from /proc/meminfo
  std::string usage_error_str_;                 //!< @brief Error string
  std::map<std::string, size_t> usage_map_;     //!< @brief Output of /proc/meminfo
  double usage_elapsed_ms_;                     //!< @brief Execution time of readUsage
  std::mutex usage_timeout_mutex_;  //!< @brief Mutex regarding timeout for executing readUsage
  bool usage_timeout_expired_;      //!< @brief Timeout for executing readUsage has expired or not

  std::mutex ecc_mutex_;             //!< @brief Mutex for output from edac-util command
  std::string ecc_error_str_;        //!< @brief Error string
  std::string ecc_pipe2_error_str_;  //!< @brief Error string regarding pipe2 function call
  std::string ecc_output_;           //!< @brief Output of edac-util command
  double ecc_elapsed_ms_;            //!< @brief Execution time of edac-util command
  std::mutex
    ecc_timeout_mutex_;       //!< @brief Mutex regarding timeout for executing edac-util command
  bool ecc_timeout_expired_;  //!< @brief Timeout for executing edac-util command has expired or not
  bool use_edac_util_;        //!< @brief Available to use edac-util command or not

  /**
   * @brief Memory usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "very high load"}};
};

#endif  // SYSTEM_MONITOR__MEM_MONITOR__MEM_MONITOR_HPP_
