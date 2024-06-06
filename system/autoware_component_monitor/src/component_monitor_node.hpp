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

using msg_t = autoware_internal_msgs::msg::SystemUsage;
using fields_t = std::vector<std::vector<std::string>>;

namespace autoware::component_monitor
{

class ComponentMonitor : public rclcpp::Node
{
public:
  explicit ComponentMonitor(const rclcpp::NodeOptions & node_options);

private:
  void monitor(const pid_t & pid) const;
  fields_t get_stats(const pid_t & pid) const;
  std::stringstream run_command(const std::string & cmd) const;
  static fields_t get_fields(std::stringstream & std_out);
  static float get_cpu_percentage(const fields_t & fields);
  static std::pair<uint64_t, uint64_t> get_system_memory(const fields_t & fields);
  static std::pair<uint64_t, float> get_process_memory(fields_t & fields);
  static float to_float(const std::string & str);
  // cSpell:ignore mebibytes, gibibytes, tebibytes, pebibytes, exbibytes
  static uint64_t to_uint64(const std::string & str);
  static uint64_t kib_to_bytes(uint64_t kibibytes);
  static uint64_t mib_to_bytes(uint64_t mebibytes);
  static uint64_t gib_to_bytes(uint64_t gibibytes);
  static uint64_t tib_to_bytes(uint64_t tebibytes);
  static uint64_t pib_to_bytes(uint64_t pebibytes);
  static uint64_t eib_to_bytes(uint64_t exbibytes);

  rclcpp::Publisher<msg_t>::SharedPtr usage_pub_;
};

}  // namespace autoware::component_monitor
// autoware
