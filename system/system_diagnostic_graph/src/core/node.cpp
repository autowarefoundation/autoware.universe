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

#include <algorithm>
#include <utility>

namespace system_diagnostic_graph
{

UnitNode::UnitNode(const KeyType & key) : key_(key)
{
  level_ = DiagnosticStatus::STALE;
}

DiagnosticNode UnitNode::report() const
{
  DiagnosticStatus status;
  status.level = level_;
  status.name = key_;
  status.hardware_id = "";

  DiagnosticNode message;
  message.status = status;
  return message;
}

void UnitNode::update()
{
  std::vector<DiagnosticLevel> levels;
  if (!links_.empty()) {
    const auto get_level = [](const auto * link) { return link->level(); };
    levels.resize(links_.size());
    std::transform(links_.begin(), links_.end(), levels.begin(), get_level);
  }
  level_ = expr_->exec(levels);
}

void UnitNode::create(DiagGraphInit & graph, const UnitConfig & config)
{
  for (const auto & link : config.expr.list) {
    BaseNode * node = graph.get(link);
    if (!node) {
      throw ConfigError("unknown unit name: " + config.hint);
    }
    links_.push_back(node);
  }
  expr_ = BaseExpr::create(config.expr.type);
}

DiagNode::DiagNode(const KeyType & key) : key_(key)
{
  level_ = DiagnosticStatus::STALE;
}

DiagnosticNode DiagNode::report() const
{
  DiagnosticStatus status = status_;
  status.level = level_;
  status.name = key_.first;
  status.hardware_id = key_.second;

  DiagnosticNode message;
  message.status = status;
  return message;
}

void DiagNode::update()
{
  // TODO(Takagi, Isamu): timeout, error hold
  // constexpr double timeout = 1.0; // TODO(Takagi, Isamu): parameterize
}

void DiagNode::callback(const DiagnosticStatus & status)
{
  level_ = status.level;
  status_ = status;
}

UnitNode * DiagGraphData::make_unit(const std::string & name)
{
  const auto key = name;
  auto unit = unit_list.emplace_back(std::make_unique<UnitNode>(key)).get();
  unit_dict[key] = unit;
  return unit;
}

UnitNode * DiagGraphData::find_unit(const std::string & name)
{
  const auto key = name;
  return unit_dict.count(key) ? unit_dict.at(key) : nullptr;
}

DiagNode * DiagGraphData::make_leaf(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  auto leaf = leaf_list.emplace_back(std::make_unique<DiagNode>(key)).get();
  leaf_dict[key] = leaf;
  return leaf;
}

DiagNode * DiagGraphData::find_leaf(const std::string & name, const std::string & hardware)
{
  const auto key = std::make_pair(name, hardware);
  return leaf_dict.count(key) ? leaf_dict.at(key) : nullptr;
}

BaseNode * DiagGraphInit::get(const LinkConfig & link)
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
