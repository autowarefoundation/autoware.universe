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

#include <string>

#include <ros/ros.h>

#include <dummy_diag_publisher/DummyDiagPublisherConfig.h>
#include <dynamic_reconfigure/server.h>

#include <diagnostic_updater/diagnostic_updater.h>

struct DiagConfig
{
  DiagConfig() = default;
  explicit DiagConfig(XmlRpc::XmlRpcValue value)
  : name(static_cast<std::string>(value["name"])),
    hardware_id(static_cast<std::string>(value["hardware_id"])),
    msg_ok(static_cast<std::string>(value["msg_ok"])),
    msg_warn(static_cast<std::string>(value["msg_warn"])),
    msg_error(static_cast<std::string>(value["msg_error"])),
    msg_stale(static_cast<std::string>(value["msg_stale"]))
  {
  }

  std::string name;
  std::string hardware_id;
  std::string msg_ok;
  std::string msg_warn;
  std::string msg_error;
  std::string msg_stale;
};

class DummyDiagPublisherNode
{
public:
  DummyDiagPublisherNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  DiagConfig diag_config_;

  // Dynamic Reconfigure
  void onConfig(
    const dummy_diag_publisher::DummyDiagPublisherConfig & config, const uint32_t level);

  dynamic_reconfigure::Server<dummy_diag_publisher::DummyDiagPublisherConfig> dynamic_reconfigure_;
  dummy_diag_publisher::DummyDiagPublisherConfig config_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Timer
  void onTimer(const ros::TimerEvent & event);
  ros::Timer timer_;
};
