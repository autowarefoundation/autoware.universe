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

#include <autoware_error_monitor/autoware_error_monitor_node.h>

#include <fmt/format.h>

void AutowareErrorMonitorNode::loadRequiredConditions(const std::string & key)
{
  const auto param_key = std::string("required_conditions") + key;

  RequiredConditions value;
  if (!private_nh_.getParam(param_key, value)) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  required_conditions_map_.insert(std::make_pair(key, value));
}

void AutowareErrorMonitorNode::onDiagArray(const diagnostic_msgs::DiagnosticArray::ConstPtr & msg)
{
  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
      diag_buffer_map_.insert(std::make_pair(diag.name, DiagBuffer{}));
    }

    auto & diag_buffer = diag_buffer_map_.at(diag.name);
    diag_buffer.push_back(DiagStamped{header, diag});

    while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }
}

void AutowareErrorMonitorNode::onTimer(const ros::TimerEvent & event)
{
  autoware_system_msgs::DrivingCapability driving_capability;

  driving_capability.manual_driving = judgeCapability(KeyName::manual_driving);
  driving_capability.autonomous_driving = judgeCapability(KeyName::autonomous_driving);
  driving_capability.remote_control = judgeCapability(KeyName::remote_control);
  driving_capability.safe_stop = judgeCapability(KeyName::safe_stop);
  driving_capability.emergency_stop = judgeCapability(KeyName::emergency_stop);

  pub_driving_capability_.publish(driving_capability);
}

bool AutowareErrorMonitorNode::judgeCapability(const std::string & key)
{
  for (const auto & required_condition : required_conditions_map_.at(key)) {
    const auto diag_name = fmt::format("/{}", required_condition);
    const auto & latest_diag = getLatestDiag(diag_name);

    if (!latest_diag) {
      return false;
    }

    if (latest_diag->status.level != diagnostic_msgs::DiagnosticStatus::OK) {
      return false;
    }

    const auto time_diff = ros::Time::now() - latest_diag->header.stamp;
    if (time_diff.toSec() > diag_timeout_sec_) {
      return false;
    }
  }

  return true;
}

boost::optional<DiagStamped> AutowareErrorMonitorNode::getLatestDiag(const std::string & diag_name)
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }

  const auto & diag_buffer = diag_buffer_map_.at(diag_name);

  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}

AutowareErrorMonitorNode::AutowareErrorMonitorNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  loadRequiredConditions(KeyName::manual_driving);
  loadRequiredConditions(KeyName::autonomous_driving);
  loadRequiredConditions(KeyName::remote_control);
  loadRequiredConditions(KeyName::safe_stop);
  loadRequiredConditions(KeyName::emergency_stop);

  // Subscriber
  sub_diag_array_ =
    private_nh_.subscribe("input/diag_array", 1, &AutowareErrorMonitorNode::onDiagArray, this);

  // Publisher
  pub_driving_capability_ =
    private_nh_.advertise<autoware_system_msgs::DrivingCapability>("output/driving_capability", 1);

  // Timer
  timer_ =
    private_nh_.createTimer(ros::Rate(update_rate_), &AutowareErrorMonitorNode::onTimer, this);
}
