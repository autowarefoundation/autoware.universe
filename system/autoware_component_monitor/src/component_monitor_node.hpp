// Copyright 2024 The Autoware Foundation
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

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_msgs/msg/system_usage.hpp"
#include "std_msgs/msg/header.hpp"

#include "boost/filesystem.hpp"
#include "boost/process.hpp"

#include <string>
#include <utility>
#include <vector>

namespace bp = boost::process;
namespace fs = boost::filesystem;

namespace autoware::component_monitor
{

class ComponentMonitor : public rclcpp::Node
{
public:
  explicit ComponentMonitor(const rclcpp::NodeOptions & node_options);

private:
  void monitor();
  void publish();
  void get_stats();
  std::stringstream run_command(const std::string & cmd) const;
  static std::vector<std::string> get_fields(std::stringstream & std_out);
  void get_cpu_usage();
  void get_mem_usage();
  static float to_float(const std::string & str);
  // cSpell:ignore mebibytes, gibibytes, tebibytes, pebibytes, exbibytes
  static uint32_t to_uint32(const std::string & str);
  static uint64_t mib_to_kib(uint64_t mebibytes);
  static uint64_t gib_to_kib(uint64_t gibibytes);
  static uint64_t tib_to_kib(uint64_t tebibytes);
  static uint64_t pib_to_kib(uint64_t pebibytes);
  static uint64_t eib_to_kib(uint64_t exbibytes);

  rclcpp::Publisher<autoware_internal_msgs::msg::SystemUsage>::SharedPtr usage_pub_;

  pid_t pid_;
  std::vector<std::string> fields_{};
  autoware_internal_msgs::msg::SystemUsage usage_msg_{};
};

}  // namespace autoware::component_monitor
// autoware
