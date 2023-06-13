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

DiagUnit::DiagUnit(const KeyType & key) : key_(key)
{
}

std::vector<DiagNode *> DiagUnit::create(DiagGraphInit & graph, const UnitConfig & config)
{
  (void)graph;
  (void)config;

  RCLCPP_INFO_STREAM(logger, config.name << ": " << config.expr.type);
  for (const auto & link : config.expr.list) {
    graph.get(link);
  }
  return {};
}

DiagLeaf::DiagLeaf(const KeyType & key) : key_(key)
{
  level_ = DiagnosticStatus::STALE;
}

DiagnosticNode DiagLeaf::report()
{
  return DiagnosticNode();
}

void DiagLeaf::update(const DiagnosticStatus & status)
{
  level_ = status.level;
}

DiagUnit * DiagGraphData::make_unit(const std::string & name)
{
  const auto key = name;
  auto unit = unit_list.emplace_back(std::make_unique<DiagUnit>(key)).get();
  unit_dict[key] = unit;
  return unit;
}

DiagUnit * DiagGraphData::find_unit(const std::string & name)
{
  const auto key = name;
  return unit_dict.count(key) ? unit_dict.at(key) : nullptr;
}

DiagLeaf * DiagGraphData::make_leaf(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  auto leaf = leaf_list.emplace_back(std::make_unique<DiagLeaf>(key)).get();
  leaf_dict[key] = leaf;
  return leaf;
}

DiagLeaf * DiagGraphData::find_leaf(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  return leaf_dict.count(key) ? leaf_dict.at(key) : nullptr;
}

DiagNode * DiagGraphInit::get(const LinkConfig & link)
{
  // For link to unit type.
  if (link.is_unit_type) {
    return data_.find_unit(link.name);
  }
  // For link to diag type.
  auto * leaf = data_.find_leaf(link.name, link.hardware);
  if (leaf) return leaf;
  return data_.make_leaf(link.name, link.hardware);
}

}  // namespace system_diagnostic_graph
