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

#include <autoware_state_monitor/autoware_state_monitor_node.h>

#include <numeric>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_state_monitor/rosconsole_wrapper.h>

void AutowareStateMonitorNode::setupDiagnosticUpdater()
{
  updater_.setHardwareID("autoware_state_monitor");

  std::vector<std::string> module_names;
  private_nh_.param("module_names", module_names, {});

  // Topic
  for (const auto & module_name : module_names) {
    const auto diag_name = fmt::format("{}_topic_status", module_name);

    updater_.add(
      diag_name, boost::bind(&AutowareStateMonitorNode::checkTopicStatus, this, _1, module_name));
  }

  // TF
  updater_.add(
    "localization_tf_status",
    boost::bind(&AutowareStateMonitorNode::checkTfStatus, this, _1, "localization"));
}

void AutowareStateMonitorNode::checkTopicStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::vector<std::string> error_msgs;

  const auto & topic_stats = state_input_.topic_stats;
  const auto & tf_stats = state_input_.tf_stats;

  // Check topic received
  for (const auto & topic_config : topic_stats.non_received_list) {
    if (topic_config.module != module_name) {
      continue;
    }

    const auto msg = fmt::format("topic `{}` is not received", topic_config.name);

    error_msgs.push_back(msg);
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  // Check topic rate
  for (const auto & topic_config_pair : topic_stats.slow_rate_list) {
    const auto & topic_config = topic_config_pair.first;
    const auto & topic_rate = topic_config_pair.second;

    if (topic_config.module != module_name) {
      continue;
    }

    const auto msg = fmt::format(
      "topic `{}` is slow rate: warn_rate = {}, actual_rate = {}", topic_config.name,
      topic_config.warn_rate, topic_rate);

    error_msgs.push_back(msg);
    level = diagnostic_msgs::DiagnosticStatus::WARN;
  }

  // Check topic timeout
  for (const auto & topic_config_pair : topic_stats.timeout_list) {
    const auto & topic_config = topic_config_pair.first;
    const auto & last_received_time = topic_config_pair.second;

    if (topic_config.module != module_name) {
      continue;
    }

    const auto msg = fmt::format(
      "topic `{}` is timeout: timeout = {}, checked_time = {:10.3f}, last_received_time = {:10.3f}",
      topic_config.name, topic_config.timeout, topic_stats.checked_time.toSec(),
      last_received_time.toSec());

    error_msgs.push_back(msg);
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  // Create message
  std::ostringstream oss;
  for (const auto & msg : error_msgs) {
    oss << msg << std::endl;
  }

  stat.summary(level, oss.str());
}

void AutowareStateMonitorNode::checkTfStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & module_name)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::vector<std::string> error_msgs;

  const auto & topic_stats = state_input_.topic_stats;
  const auto & tf_stats = state_input_.tf_stats;

  // Check tf timeout
  for (const auto & tf_config_pair : tf_stats.timeout_list) {
    const auto & tf_config = tf_config_pair.first;
    const auto & last_received_time = tf_config_pair.second;

    if (tf_config.module != module_name) {
      continue;
    }

    const auto msg = fmt::format(
      "tf from `{}` to `{}` is timeout: timeout = {}, checked_time = {:10.3f}, last_received_time "
      "= {:10.3f}",
      tf_config.from, tf_config.to, tf_config.timeout, tf_stats.checked_time.toSec(),
      last_received_time.toSec());

    error_msgs.push_back(msg);
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

  // Create message
  std::ostringstream oss;
  for (const auto & msg : error_msgs) {
    oss << msg << std::endl;
  }

  stat.summary(level, oss.str());
}
