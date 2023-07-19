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
  topological_sort();

  // Fix the node index for the ros message.
  std::vector<NodeWithIndexLink> nodes;
  for (const auto & leaf : data_.leaf_list) {
    nodes.push_back(NodeWithIndexLink{leaf.get(), {}});
  }
  for (const auto & unit : data_.unit_list) {
    nodes.push_back(NodeWithIndexLink{unit.get(), {}});
  }

  // Set the link index for the ros message.
  std::unordered_map<DiagNode *, size_t> indices;
  for (size_t i = 0; i < nodes.size(); ++i) {
    indices[nodes[i].node] = i;
  }
  for (auto & node : nodes) {
    for (const auto & link : node.node->links()) {
      node.links.push_back(indices.at(link));
    }
  }
  index_nodes_ = nodes;

  for (size_t i = 0; i < nodes.size(); ++i) {
    std::cout << std::setw(2) << i << ": " << nodes[i].node->name();
    for (const auto & link : nodes[i].links) {
      std::cout << " " << link;
    }
    std::cout << std::endl;
  }

  return DiagnosticGraph();
}

DiagnosticArray DiagGraph::report(const rclcpp::Time & stamp)
{
  DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.reserve(data_.leaf_list.size() + data_.unit_list.size());

  for (const auto & node : index_nodes_) {
    node.node->update();
  }

  for (const auto & node : index_nodes_) {
    array.status.push_back(node.node->report());
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

void DiagGraph::topological_sort()
{
  std::unordered_map<DiagNode *, int> degrees;
  std::vector<DiagNode *> nodes;
  std::vector<DiagNode *> result;
  std::vector<DiagNode *> buffer;

  // Create a list of unit nodes and leaf nodes.
  for (const auto & unit : data_.unit_list) {
    nodes.push_back(unit.get());
  }
  for (const auto & leaf : data_.leaf_list) {
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
    const auto node = buffer.back();
    buffer.pop_back();
    for (const auto & link : node->links()) {
      if (--degrees[link] == 0) {
        buffer.push_back(link);
      }
    }
    result.push_back(node);
  }

  // Cyclic nodes are not included in the result.
  if (result.size() != nodes.size()) {
    throw ConfigError("detect graph circulation");
  }

  // Create indices to update original vector.
  std::unordered_map<DiagNode *, size_t> indices;
  for (size_t i = 0; i < data_.unit_list.size(); ++i) {
    indices[data_.unit_list[i].get()] = i;
  }

  // Update original vector.
  std::vector<std::unique_ptr<DiagUnit>> unit_list;
  for (const auto & node : result) {
    if (indices.count(node)) {
      unit_list.push_back(std::move(data_.unit_list[indices[node]]));
    }
  }
  std::reverse(unit_list.begin(), unit_list.end());
  data_.unit_list = std::move(unit_list);
}

}  // namespace system_diagnostic_graph
