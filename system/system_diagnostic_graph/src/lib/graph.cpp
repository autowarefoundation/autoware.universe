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

#include <yaml-cpp/yaml.h>

#include <string>

namespace system_diagnostic_graph
{

void DiagGraph::create(const std::string & file)
{
  YAML::Node yaml = YAML::LoadFile(file);
  std::cout << YAML::Dump(yaml) << std::endl;
}

DiagnosticGraph DiagGraph::report(const rclcpp::Time & stamp)
{
  DiagnosticGraph graph;
  graph.stamp = stamp;
  graph.nodes.reserve(nodes_.size() + diags_.size());

  for (const auto & node : nodes_) {
    graph.nodes.push_back(node->report());
  }
  for (const auto & diag : diags_) {
    graph.nodes.push_back(diag.second->report());
  }
  return graph;
}

void DiagGraph::update(const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    const auto key = DiagLeaf::get_key(status);
    if (!diags_.count(key)) {
      diags_.emplace(key, std::make_shared<DiagLeaf>(status));
    }
    diags_.at(key)->update(status);
  }
}

}  // namespace system_diagnostic_graph
