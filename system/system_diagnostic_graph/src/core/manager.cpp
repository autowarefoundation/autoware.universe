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
#include "expr.hpp"
#include "node.hpp"

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

GraphManager::GraphManager()
{
  // for unique_ptr
}

GraphManager::~GraphManager()
{
  // for unique_ptr
}

void GraphManager::init(const std::string & file, const std::string & mode)
{
  ConfigFile root = load_config_root(file);

  std::vector<std::unique_ptr<BaseNode>> nodes;
  std::unordered_map<std::string, BaseNode *> paths;

  ExprInit exprs(mode);

  for (auto & config : root.units) {
    if (config.mode.check(mode)) {
      nodes.push_back(std::make_unique<UnitNode>(config.path));
    }
  }

  for (auto & config : root.diags) {
    if (config.mode.check(mode)) {
      nodes.push_back(std::make_unique<DiagNode>(config.path));
    }
  }

  for (const auto & node : nodes) {
    paths[node->path()] = node.get();
  }

  for (auto & config : root.units) {
    if (paths.count(config.path)) {
      paths.at(config.path)->create(config.dict, exprs);
    }
  }

  for (auto & config : root.diags) {
    if (paths.count(config.path)) {
      paths.at(config.path)->create(config.dict, exprs);
    }
  }

  for (auto & [link, config] : exprs.get()) {
    link->init(config, paths);
  }

  for (const auto & node : nodes) {
    std::cout << node->path() << std::endl;
  }
  // Sort unit nodes in topological order for update dependencies.
  // graph_.topological_sort();

  // Set the link index for the ros message.
  /*
  const auto & nodes = graph_.nodes();
  for (size_t i = 0; i < nodes.size(); ++i) {
    nodes[i]->set_index(i);
  }
  */

  nodes_ = std::move(nodes);
}

void GraphManager::callback(const DiagnosticArray & array, const rclcpp::Time & stamp)
{
  (void)array;
  (void)stamp;
  /*
  for (const auto & status : array.status) {
    auto diag = graph_.find_diag(status.name, status.hardware_id);
    if (diag) {
      diag->callback(status, stamp);
    } else {
      // TODO(Takagi, Isamu): handle unknown diagnostics
    }
  }
  */
}

void GraphManager::update(const rclcpp::Time & stamp)
{
  for (const auto & node : nodes_) {
    node->update(stamp);
  }
  stamp_ = stamp;
}

DiagnosticGraph GraphManager::create_graph_message() const
{
  DiagnosticGraph message;
  message.stamp = stamp_;
  message.nodes.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    message.nodes.push_back(node->report());
  }
  return message;
}

}  // namespace system_diagnostic_graph
