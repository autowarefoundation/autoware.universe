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

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

void DiagGraph::create(const std::string & file)
{
  struct TempUnit
  {
    UnitConfig config;
    DiagUnit * node;
    std::vector<DiagNode *> links;
  };

  const auto configs = load_config_file(file);
  DiagGraphData data;
  DiagGraphInit init(data);

  std::vector<TempUnit> units;
  for (const auto & config : configs) {
    TempUnit unit;
    unit.config = config;
    unit.node = data.make_unit(config.name);
    units.push_back(unit);
  }

  for (auto & unit : units) {
    unit.links = unit.node->create(init, unit.config);
  }

  data_ = std::move(data);
}

DiagnosticGraph DiagGraph::report(const rclcpp::Time & stamp)
{
  DiagnosticGraph graph;
  graph.stamp = stamp;
  graph.nodes.reserve(data_.leaf_list.size() + data_.unit_list.size());

  for (const auto & leaf : data_.leaf_list) {
    graph.nodes.push_back(leaf->report());
  }
  for (const auto & unit : data_.unit_list) {
    graph.nodes.push_back(unit->report());
  }
  return graph;
}

void DiagGraph::update(const DiagnosticArray & array)
{
  (void)array;
  /*
  for (const auto & status : array.status) {
    const auto key = DiagLeaf::get_key(status);
    if (!data_.leaf_dict.count(key)) {
      data_.leaf_dict.emplace(key, std::make_shared<DiagLeaf>(status));
    }
    data_.leaf_dict.at(key)->update(status);
  }
  */
}

}  // namespace system_diagnostic_graph
