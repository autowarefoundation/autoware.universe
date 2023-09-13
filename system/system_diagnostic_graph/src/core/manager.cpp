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

#include "manager.hpp"

#include "config.hpp"

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

UnitNode * find_node(Graph & graph, const std::string & name)
{
  const auto node = graph.find_unit(name);
  if (!node) {
    throw ConfigError("summary node '" + name + "' does node exist");
  }
  return node;
};

void GraphManager::init(const std::string & file)
{
  const auto configs = load_config_file(file);

  // Create unit nodes first because it is necessary for the link.
  std::vector<std::pair<NodeConfig, UnitNode *>> units;
  for (const auto & config : configs) {
    UnitNode * unit = graph_.make_unit(config->name);
    units.push_back(std::make_pair(config, unit));
  }

  // Reflect the config after creating all the unit nodes,
  for (auto & [config, unit] : units) {
    unit->create(graph_, config);
  }

  // Sort unit nodes in topological order for update dependencies.
  graph_.topological_sort();

  // Set the link index for the ros message.
  const auto & nodes = graph_.nodes();
  for (size_t i = 0; i < nodes.size(); ++i) {
    nodes[i]->set_index(i);
  }

  // Get reserved unit node for summary.
  modes_.stop_mode = find_node(graph_, "/autoware/operation/stop");
  modes_.autonomous_mode = find_node(graph_, "/autoware/operation/autonomous");
  modes_.local_mode = find_node(graph_, "/autoware/operation/local");
  modes_.remote_mode = find_node(graph_, "/autoware/operation/remote");
  modes_.emergency_stop_mrm = find_node(graph_, "/autoware/operation/emergency-stop");
  modes_.comfortable_stop_mrm = find_node(graph_, "/autoware/operation/comfortable-stop");
  modes_.pull_over_mrm = find_node(graph_, "/autoware/operation/pull-over");
}

void GraphManager::callback(const DiagnosticArray & array, const rclcpp::Time & stamp)
{
  for (const auto & status : array.status) {
    auto diag = graph_.find_diag(status.name, status.hardware_id);
    if (diag) {
      diag->callback(status, stamp);
    } else {
      // TODO(Takagi, Isamu): handle unknown diagnostics
    }
  }
}

void GraphManager::update(const rclcpp::Time & stamp)
{
  for (const auto & node : graph_.nodes()) {
    node->update(stamp);
  }
  stamp_ = stamp;
}

DiagnosticGraph GraphManager::create_graph_message() const
{
  DiagnosticGraph message;
  message.stamp = stamp_;
  message.nodes.reserve(graph_.nodes().size());
  for (const auto & node : graph_.nodes()) {
    message.nodes.push_back(node->report());
  }
  return message;
}

OperationModeAvailability GraphManager::create_modes_message() const
{
  const auto is_ok = [](const UnitNode * node) { return node->level() == DiagnosticStatus::OK; };

  OperationModeAvailability message;
  message.stamp = stamp_;
  message.stop = is_ok(modes_.stop_mode);
  message.autonomous = is_ok(modes_.autonomous_mode);
  message.local = is_ok(modes_.local_mode);
  message.remote = is_ok(modes_.remote_mode);
  message.emergency_stop = is_ok(modes_.emergency_stop_mrm);
  message.comfortable_stop = is_ok(modes_.comfortable_stop_mrm);
  message.pull_over = is_ok(modes_.pull_over_mrm);
  return message;
}

}  // namespace system_diagnostic_graph
