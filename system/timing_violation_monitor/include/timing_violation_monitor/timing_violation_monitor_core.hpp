// Copyright 2022 TIER IV, Inc.
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

#ifndef TIMING_VIOLATION_MONITOR__TIMING_VIOLATION_MONITOR_CORE_HPP_
#define TIMING_VIOLATION_MONITOR__TIMING_VIOLATION_MONITOR_CORE_HPP_

#include "builtin_interfaces/msg/time.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace timing_violation_monitor
{
struct DeadlineTimer
{
  int64_t self_j;
  double start_time;
  double timer_val;
  rclcpp::TimerBase::SharedPtr timer;
  uint64_t uniq;
  bool valid;
};

using DeadlineTimerMap = std::unordered_map<uint64_t, DeadlineTimer>;

enum class e_stat {
  ST_NONE,
  ST_INIT,
  ST_DETECT,
};

class TimingViolationMonitorPathConfig
{
public:
  uint32_t index;
  std::string path_name;
  std::string topic;
  std::string mtype;
  double p_i;
  double d_i;
  std::string level;
  std::mutex * p_mutex;

  TimingViolationMonitorPathConfig(uint32_t index, std::mutex * mtx) : index(index), p_mutex(mtx)
  {
    status = e_stat::ST_NONE;
    cur_j = 0l;
    completed_j = -1l;
    deadline_miss_count = 0lu;
    deadline_timer_manage = 0lu;
    r_i_j_1 = r_i_j = 0.0;
  }
  TimingViolationMonitorPathConfig(const TimingViolationMonitorPathConfig & c) = delete;
  ~TimingViolationMonitorPathConfig() { delete p_mutex; }

  // variables
  e_stat status;
  int64_t cur_j;
  int64_t completed_j;
  double periodic_timer_val;
  rclcpp::TimerBase::SharedPtr periodic_timer;
  rclcpp::TimerBase::SharedPtr interval_timer;
  DeadlineTimerMap deadline_timer;
  builtin_interfaces::msg::Time r_i_j_1_stamp;
  double r_i_j_1;
  double r_i_j;
  uint64_t deadline_timer_manage;
  // diagnostics
  uint64_t violation_count_threshold;
  uint64_t deadline_miss_count;
  uint64_t prev_deadline_miss_count;
};

class TimingViolationMonitor : public rclcpp::Node
{
public:
  TimingViolationMonitor();
  bool get_debug_param() { return params_.debug_ctrl; }
  void registerNodeToDebug(const std::shared_ptr<TimingViolationMonitor> & node);
  double get_now();

private:
  struct Parameters
  {
    bool debug_ctrl;
  };
  Parameters params_{};

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  void loadTargetPaths();
  // Subscriber
  void onGenTopic(
    const std::shared_ptr<rclcpp::SerializedMessage> msg,
    std::shared_ptr<TimingViolationMonitorPathConfig> pinfo_ptr);
  void topicCallback(
    TimingViolationMonitorPathConfig & pinfo, double & pub_time, double & cur_ros,
    double & response_time);
  bool isOverDeadline(
    TimingViolationMonitorPathConfig & pinfo, double & pub_time, double & cur_ros,
    double & response_time);
  void checkExistingTimers(TimingViolationMonitorPathConfig & pinfo);
  void restartTimers(TimingViolationMonitorPathConfig & pinfo, double & cur_ros);
  // Timer
  void onIntervalTimer(TimingViolationMonitorPathConfig & pinfo);
  void onPeriodicTimer(TimingViolationMonitorPathConfig & pinfo);
  void onDeadlineTimer(TimingViolationMonitorPathConfig & pinfo, DeadlineTimer & dm);
  void startIntervalTimer(TimingViolationMonitorPathConfig & pinfo, double & time_val);
  void startPeriodicTimer(TimingViolationMonitorPathConfig & pinfo, double & time_val);
  void startDeadlineTimer(
    TimingViolationMonitorPathConfig & pinfo, double & start_time, double & time_val);
  bool isValidDeadlineTimer(TimingViolationMonitorPathConfig & pinfo, DeadlineTimer & dm);

  void diagDataUpdate(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // debug
  void stopDetect(TimingViolationMonitorPathConfig & pinfo);
};

}  // namespace timing_violation_monitor
#endif  // TIMING_VIOLATION_MONITOR__TIMING_VIOLATION_MONITOR_CORE_HPP_
