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

#include "graph.hpp"

#include "config.hpp"

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

DiagnosticGraph DiagGraph::create(const std::string & file)
{
  const auto configs = load_config_file(file);
  data_ = DiagGraphData();

  // Create nodes first because it is necessary for the link.
  std::vector<std::pair<UnitConfig, DiagUnit *>> units;
  for (const auto & config : configs) {
    DiagUnit * unit = data_.make_unit(config.name);
    units.push_back(std::make_pair(config, unit));
  }

  // Reflect the config after creating all the nodes,
  DiagGraphInit graph(data_);
  for (auto & [config, unit] : units) {
    unit->create(graph, config);
  }

  // Sort unit nodes in topological order for update dependencies.
  topological_nodes_ = topological_sort(data_);

  // Set the link index for the ros message.
  for (size_t i = 0; i < topological_nodes_.size(); ++i) {
    topological_nodes_[i]->set_index(i);
  }

  // Create graph structure message.
  DiagnosticGraph message;
  for (const auto & node : topological_nodes_) {
    message.nodes.push_back(node->struct_message());
  }
  return message;
}

DiagnosticArray DiagGraph::report(const rclcpp::Time & stamp)
{
  DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.reserve(topological_nodes_.size());

  for (const auto & node : topological_nodes_) {
    node->update();
  }
  for (const auto & node : topological_nodes_) {
    array.status.push_back(node->report());
  }
  return array;
}

void DiagGraph::callback(const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    auto leaf = data_.find_leaf(status.name, status.hardware_id);
    if (leaf) {
      leaf->callback(status);
    } else {
      // TODO(Takagi, Isamu): handle unknown diagnostics
    }
  }
}

std::vector<DiagNode *> DiagGraph::topological_sort(const DiagGraphData & data)
{
  std::unordered_map<DiagNode *, int> degrees;
  std::deque<DiagNode *> nodes;
  std::deque<DiagNode *> result;
  std::deque<DiagNode *> buffer;

  // Create a list of unit nodes and leaf nodes.
  for (const auto & unit : data.unit_list) {
    nodes.push_back(unit.get());
  }
  for (const auto & leaf : data.leaf_list) {
    nodes.push_back(leaf.get());
  }

  // Count degrees of each node.
  for (const auto & node : nodes) {
    for (const auto & link : node->links()) {
      ++degrees[link];
    }
  }

  // Find initial nodes that are zero degrees.
  for (const auto & node : nodes) {
    if (degrees[node] == 0) {
      buffer.push_back(node);
    }
  }

  // Sort by topological order.
  while (!buffer.empty()) {
    const auto node = buffer.front();
    buffer.pop_front();
    for (const auto & link : node->links()) {
      if (--degrees[link] == 0) {
        buffer.push_back(link);
      }
    }
    result.push_back(node);
  }

  // Detect circulation because the result does not include the nodes on the loop.
  if (result.size() != nodes.size()) {
    throw ConfigError("detect graph circulation");
  }

  return std::vector<DiagNode *>(result.rbegin(), result.rend());
}

}  // namespace system_diagnostic_graph
