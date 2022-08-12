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

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <string>
#include <unordered_map>

namespace default_ad_api
{

class DiagnosticsMonitor
{
public:
  explicit DiagnosticsMonitor(rclcpp::Node * node);
  bool is_ok();

private:
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticLevel = DiagnosticArray::_status_type::value_type::_level_type;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_diag_;
  std::unordered_map<std::string, DiagnosticLevel> levels_;
  void on_diag(const DiagnosticArray::ConstSharedPtr msg);
};

}  // namespace default_ad_api

#endif  // DIAGNOSTICS_HPP_
