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
#include "error.hpp"
#include "units.hpp"

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// DEBUG
#include <iostream>

namespace system_diagnostic_graph
{

BaseUnit::UniquePtrList topological_sort(BaseUnit::UniquePtrList && input)
{
  return std::move(input);

  /*
  std::unordered_map<BaseNode *, int> degrees;
  std::deque<BaseNode *> nodes;
  std::deque<BaseNode *> result;
  std::deque<BaseNode *> buffer;

  // Create a list of raw pointer nodes.
  for (const auto & node : input) {
    nodes.push_back(node.get());
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

  // Reverse the result to process from leaf node.
  result = std::deque<BaseNode *>(result.rbegin(), result.rend());

  // Replace node vector.
  std::unordered_map<BaseNode *, size_t> indices;
  for (size_t i = 0; i < result.size(); ++i) {
    indices[result[i]] = i;
  }
  std::vector<std::unique_ptr<BaseNode>> sorted(input.size());
  for (auto & node : input) {
    sorted[indices[node.get()]] = std::move(node);
  }
  input = std::move(sorted);
  */
}

BaseUnit::UniquePtr make_node(const UnitConfig::SharedPtr & config)
{
  if (config->type == "diag") {
    return std::make_unique<DiagUnit>(config->path);
  }
  if (config->type == "link") {
    return std::make_unique<LinkUnit>(config->path);
  }
  if (config->type == "and") {
    return std::make_unique<AndUnit>(config->path, false);
  }
  if (config->type == "short-circuit-and") {
    return std::make_unique<AndUnit>(config->path, true);
  }
  if (config->type == "or") {
    return std::make_unique<OrUnit>(config->path);
  }
  if (config->type == "debug-ok") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::OK);
  }
  if (config->type == "debug-warn") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::WARN);
  }
  if (config->type == "debug-error") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::ERROR);
  }
  if (config->type == "debug-stale") {
    return std::make_unique<DebugUnit>(config->path, DiagnosticStatus::STALE);
  }
  throw ConfigError("unknown node type: " + config->type + " " + "TODO");
}

Graph::Graph()
{
  // for unique_ptr
}

Graph::~Graph()
{
  // for unique_ptr
}

void Graph::init(const std::string & file, const std::string & mode)
{
  (void)mode;  // TODO(Takagi, Isamu)

  BaseUnit::UniquePtrList nodes;
  BaseUnit::NodeDict dict;

  for (const auto & config : load_config_root(file).nodes) {
    const auto node = nodes.emplace_back(make_node(config)).get();
    dict.configs[config] = node;
    dict.paths[config->path] = node;
  }
  dict.paths.erase("");

  for (const auto & [config, node] : dict.configs) {
    node->init(config, dict);
  }

  // DEBUG
  for (size_t index = 0; index < nodes.size(); ++index) {
    nodes[index]->set_index(index);
  }

  for (const auto & [config, node] : dict.configs) {
    std::cout << std::left << std::setw(3) << node->index();
    std::cout << std::left << std::setw(10) << config->type;
    std::cout << std::left << std::setw(40) << node->path();
    for (const auto & child : node->children()) {
      std::cout << " " << child->index();
    }
    std::cout << std::endl;
  }

  // Sort units in topological order for update dependencies.
  nodes_ = topological_sort(std::move(nodes));
}

void Graph::callback(const DiagnosticArray & array, const rclcpp::Time & stamp)
{
  for (const auto & status : array.status) {
    const auto iter = diags_.find(status.name);
    if (iter != diags_.end()) {
      iter->second->callback(status, stamp);
    } else {
      // TODO(Takagi, Isamu)
      // unknown_->callback(status, stamp);
    }
  }
}

void Graph::update(const rclcpp::Time & stamp)
{
  for (const auto & node : nodes_) {
    node->update(stamp);
  }
  stamp_ = stamp;
}

DiagnosticGraph Graph::message() const
{
  DiagnosticGraph result;
  result.stamp = stamp_;
  result.nodes.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    result.nodes.push_back(node->report());
  }
  return result;
}

std::vector<BaseUnit *> Graph::nodes() const
{
  std::vector<BaseUnit *> result;
  result.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    result.push_back(node.get());
  }
  return result;
}

}  // namespace system_diagnostic_graph
