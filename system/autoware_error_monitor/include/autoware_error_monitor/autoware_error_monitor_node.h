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

#pragma once

#include <deque>
#include <string>
#include <unordered_map>

#include <boost/optional.hpp>

#include <ros/ros.h>

#include <autoware_system_msgs/DrivingCapability.h>
#include <diagnostic_msgs/DiagnosticArray.h>

struct DiagStamped
{
  std_msgs::Header header;
  diagnostic_msgs::DiagnosticStatus status;
};

using RequiredConditions = std::vector<std::string>;
using DiagBuffer = std::deque<DiagStamped>;

struct KeyName
{
  static constexpr const char * manual_driving = "/manual_driving";
  static constexpr const char * autonomous_driving = "/autonomous_driving";
  static constexpr const char * remote_control = "/remote_control";
  static constexpr const char * safe_stop = "/safe_stop";
  static constexpr const char * emergency_stop = "/emergency_stop";
};

class AutowareErrorMonitorNode
{
public:
  AutowareErrorMonitorNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  std::unordered_map<std::string, RequiredConditions> required_conditions_map_;

  void loadRequiredConditions(const std::string & key);

  // Timer
  ros::Timer timer_;

  void onTimer(const ros::TimerEvent & event);

  // Subscriber
  ros::Subscriber sub_diag_array_;

  void onDiagArray(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg);

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;

  // Publisher
  ros::Publisher pub_driving_capability_;

  // Algorithm
  bool judgeCapability(const std::string & key);
  boost::optional<DiagStamped> getLatestDiag(const std::string & diag_name);

  const double diag_timeout_sec_ = 1.0;
};
