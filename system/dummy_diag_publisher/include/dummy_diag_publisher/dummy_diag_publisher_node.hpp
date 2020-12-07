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

#ifndef DUMMY_DIAG_PUBLISHER_DUMMY_DIAG_PUBLISHER_H_
#define DUMMY_DIAG_PUBLISHER_DUMMY_DIAG_PUBLISHER_H_ 

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

struct DiagConfig
{
  DiagConfig() = default;
  explicit DiagConfig(std::map<std::string, std::string> config)
  : name(static_cast<std::string>(config["name"])),
    hardware_id(static_cast<std::string>(config["hardware_id"])),
    msg_ok(static_cast<std::string>(config["msg_ok"])),
    msg_warn(static_cast<std::string>(config["msg_warn"])),
    msg_error(static_cast<std::string>(config["msg_error"])),
    msg_stale(static_cast<std::string>(config["msg_stale"]))
  {
  }

  std::string name;
  std::string hardware_id;
  std::string msg_ok;
  std::string msg_warn;
  std::string msg_error;
  std::string msg_stale;
};

// Create enum with ok and warn types

class DummyDiagPublisherNode : public rclcpp::Node
{
public:
  DummyDiagPublisherNode();

private:
  enum Status {
    OK,
    WARN,
    ERROR,
    STALE,
  };

  struct DummyDiagPublisherConfig
  {
    Status status;
    bool is_active;
  };

  // Parameter
  double update_rate_;
  DiagConfig diag_config_;
  DummyDiagPublisherConfig config_;

  // Dynamic Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif
