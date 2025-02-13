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

#include "availability.hpp"

#include "graph/error.hpp"
#include "graph/graph.hpp"
#include "graph/units.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

ModesAvailability::ModesAvailability(rclcpp::Node & node, const Graph & graph)
{
  pub_ = node.create_publisher<Availability>("/system/operation_mode/availability", rclcpp::QoS(1));

  using PathDict = std::unordered_map<std::string, BaseUnit *>;
  PathDict paths;
  for (const auto & unit : graph.units()) {
    paths[unit->path()] = unit;
  }

  const auto find_unit = [](const PathDict & paths, const std::string & name) {
    const auto iter = paths.find(name);
    if (iter != paths.end()) {
      return iter->second;
    }
    throw ModeNotFound(name);
  };

  // clang-format off
  stop_mode_ =            find_unit(paths, "/autoware/modes/stop");
  autonomous_mode_ =      find_unit(paths, "/autoware/modes/autonomous");
  local_mode_ =           find_unit(paths, "/autoware/modes/local");
  remote_mode_ =          find_unit(paths, "/autoware/modes/remote");
  emergency_stop_mrm_ =   find_unit(paths, "/autoware/modes/emergency_stop");
  comfortable_stop_mrm_ = find_unit(paths, "/autoware/modes/comfortable_stop");
  pull_over_mrm_ =        find_unit(paths, "/autoware/modes/pull_over");
  // clang-format on
}

void ModesAvailability::update(const rclcpp::Time & stamp) const
{
  const auto is_ok = [](const BaseUnit * unit) { return unit->level() == DiagnosticStatus::OK; };

  // clang-format off
  Availability message;
  message.stamp            = stamp;
  message.stop             = is_ok(stop_mode_);
  message.autonomous       = is_ok(autonomous_mode_);
  message.local            = is_ok(local_mode_);
  message.remote           = is_ok(remote_mode_);
  message.emergency_stop   = is_ok(emergency_stop_mrm_);
  message.comfortable_stop = is_ok(comfortable_stop_mrm_);
  message.pull_over        = is_ok(pull_over_mrm_);
  // clang-format on

  pub_->publish(message);
}

}  // namespace autoware::diagnostic_graph_aggregator
