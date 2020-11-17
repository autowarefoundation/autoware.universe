/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE_ERROR_MONITOR_CORE_H_
#define AUTOWARE_ERROR_MONITOR_CORE_H_

#include <autoware_system_msgs/msg/driving_capability.hpp>
#include <boost/optional.hpp>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

struct DiagStamped
{
  std_msgs::msg::Header header;
  diagnostic_msgs::msg::DiagnosticStatus status;
};

using RequiredConditions = std::vector<std::string>;
using DiagBuffer = std::deque<DiagStamped>;

struct KeyName
{
  static constexpr const char * manual_driving = "manual_driving";
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * remote_control = "remote_control";
  static constexpr const char * safe_stop = "safe_stop";
  static constexpr const char * emergency_stop = "emergency_stop";
};

class AutowareErrorMonitor : public rclcpp::Node
{
public:
  AutowareErrorMonitor();

private:
  // Parameter
  int update_rate_;
  std::unordered_map<std::string, RequiredConditions> required_conditions_map_;

  void loadRequiredConditions(const std::string & key);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // Subscriber
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_array_;

  void onDiagArray(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;

  // Publisher
  rclcpp::Publisher<autoware_system_msgs::msg::DrivingCapability>::SharedPtr
    pub_driving_capability_;

  // Algorithm
  bool judgeCapability(const std::string & key);
  boost::optional<DiagStamped> getLatestDiag(const std::string & diag_name);

  const double diag_timeout_sec_ = 1.0;
};

#endif
