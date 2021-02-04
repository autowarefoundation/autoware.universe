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

#ifndef AUTOWARE_ERROR_MONITOR__AUTOWARE_ERROR_MONITOR_CORE_HPP_
#define AUTOWARE_ERROR_MONITOR__AUTOWARE_ERROR_MONITOR_CORE_HPP_

#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "boost/optional.hpp"

#include "autoware_system_msgs/msg/driving_capability.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"


struct DiagStamped
{
  std_msgs::msg::Header header;
  diagnostic_msgs::msg::DiagnosticStatus status;
};

using DiagBuffer = std::deque<DiagStamped>;

using DiagLevel = std::map<std::string, std::string>;

struct DiagConfig
{
  explicit DiagConfig(const std::string & module_name, DiagLevel & diag_level)
  : name(module_name),
    sf_at(diag_level["sf_at"]),
    lf_at(diag_level["lf_at"]),
    spf_at(diag_level["spf_at"])
  {
    // Set default values
    if (sf_at == "") {sf_at = "none";}
    if (lf_at == "") {lf_at = "warn";}
    if (spf_at == "") {spf_at = "error";}
  }

  std::string name;
  std::string sf_at;
  std::string lf_at;
  std::string spf_at;
};

using RequiredModules = std::vector<DiagConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * remote_control = "remote_control";
};

class AutowareErrorMonitor : public rclcpp::Node
{
public:
  AutowareErrorMonitor();

private:
  // Parameter
  int update_rate_;
  bool ignore_missing_diagnostics_;
  bool add_leaf_diagnostics_;
  std::unordered_map<std::string, RequiredModules> required_modules_map_;

  void loadRequiredModules(const std::string & key);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  void onTimer();

  // Subscriber
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_array_;

  void onDiagArray(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_array_;

  // Publisher
  rclcpp::Publisher<autoware_system_msgs::msg::DrivingCapability>::SharedPtr
    pub_driving_capability_;

  // Algorithm
  boost::optional<DiagStamped> getLatestDiag(const std::string & diag_name);
  int getHazardLevel(const DiagConfig & required_module, const int diag_level);
  void appendHazardDiag(
    const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & diag,
    autoware_system_msgs::msg::HazardStatus * hazard_status);
  autoware_system_msgs::msg::HazardStatus judgeHazardStatus(const std::string & key);

  const double diag_timeout_sec_ = 1.0;
};

#endif  // AUTOWARE_ERROR_MONITOR__AUTOWARE_ERROR_MONITOR_CORE_HPP_
