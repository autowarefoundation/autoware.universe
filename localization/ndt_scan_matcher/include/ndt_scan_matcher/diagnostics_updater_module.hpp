// Copyright 2023 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__DIAGNOSTICS_UPDATER_MODULE_HPP_
#define NDT_SCAN_MATCHER__DIAGNOSTICS_UPDATER_MODULE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

// This class is remake of diagnostic_updater::Updater
// reference https://github.com/ros/diagnostics

class DiagnosticsUpdaterModule : public diagnostic_updater::DiagnosticTaskVector
{
public:
  DiagnosticsUpdaterModule(
    rclcpp::Node * node, const double period = 1.0,
    const std::string & prefix_diagnostic_name = "");

private:
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec);
  void update();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::string prefix_diagnostic_name_;
};

#endif  // NDT_SCAN_MATCHER__DIAGNOSTICS_UPDATER_MODULE_HPP_
