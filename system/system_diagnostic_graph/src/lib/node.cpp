// Copyright 2023 The Autoware Contributors
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

#include "node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace system_diagnostic_graph
{

const auto logger = rclcpp::get_logger("system_diagnostic_graph");

DiagUnit::DiagUnit(const ConfigNode & config)
{
  name_ = config.yaml["name"].as<std::string>();
}

DiagLeaf::Key DiagLeaf::get_key(const DiagnosticStatus & status)
{
  return std::make_pair(status.name, status.hardware_id);
}

DiagLeaf::DiagLeaf(const DiagnosticStatus & status) : key_(get_key(status))
{
  RCLCPP_INFO_STREAM(logger, "found a new diagnosis: " << status.name);
  level_ = status.level;
}

DiagnosticNode DiagLeaf::report()
{
  return DiagnosticNode();
}

void DiagLeaf::update(const DiagnosticStatus & status)
{
  level_ = status.level;
}

}  // namespace system_diagnostic_graph
