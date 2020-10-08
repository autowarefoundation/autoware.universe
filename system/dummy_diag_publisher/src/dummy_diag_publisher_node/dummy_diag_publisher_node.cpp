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

#include <dummy_diag_publisher/dummy_diag_publisher_node.h>

#include <boost/bind.hpp>

#include <fmt/format.h>

namespace
{
template <class Config>
Config getConfig(const ros::NodeHandle & nh, const std::string & config_name)
{
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(config_name, xml)) {
    throw std::runtime_error(fmt::format("no parameter found: {}", config_name));
  }

  return Config{xml};
}
}  // namespace

void DummyDiagPublisherNode::onConfig(
  const dummy_diag_publisher::DummyDiagPublisherConfig & config, const uint32_t level)
{
  config_ = config;
}

void DummyDiagPublisherNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::DiagnosticStatus status;

  if (config_.status == dummy_diag_publisher::DummyDiagPublisher_OK) {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = diag_config_.msg_ok;
  } else if (config_.status == dummy_diag_publisher::DummyDiagPublisher_WARN) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = diag_config_.msg_warn;
  } else if (config_.status == dummy_diag_publisher::DummyDiagPublisher_ERROR) {
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = diag_config_.msg_error;
  } else if (config_.status == dummy_diag_publisher::DummyDiagPublisher_STALE) {
    status.level = diagnostic_msgs::DiagnosticStatus::STALE;
    status.message = diag_config_.msg_stale;
  } else {
    throw std::runtime_error("invalid status");
  }

  stat.summary(status.level, status.message);
}

void DummyDiagPublisherNode::onTimer(const ros::TimerEvent & event)
{
  if (config_.is_active) {
    updater_.force_update();
  }
}

DummyDiagPublisherNode::DummyDiagPublisherNode()
{
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  diag_config_ = getConfig<DiagConfig>(private_nh_, "diag_config");

  // Dynamic Reconfigure
  dynamic_reconfigure_.setCallback(boost::bind(&DummyDiagPublisherNode::onConfig, this, _1, _2));

  // Diagnostic Updater
  updater_.setHardwareID(diag_config_.hardware_id);
  updater_.add(diag_config_.name, this, &DummyDiagPublisherNode::produceDiagnostics);

  // Timer
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &DummyDiagPublisherNode::onTimer, this);
}
