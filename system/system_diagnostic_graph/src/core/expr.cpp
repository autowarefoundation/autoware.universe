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

#include "expr.hpp"

#include "graph.hpp"
#include "node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

//
#include <iostream>

namespace system_diagnostic_graph
{

using LinkStatus = std::vector<std::pair<BaseNode *, bool>>;

void extend(LinkStatus & a, const LinkStatus & b)
{
  a.insert(a.end(), b.begin(), b.end());
}

void extend_false(LinkStatus & a, const LinkStatus & b)
{
  for (const auto & p : b) {
    a.push_back(std::make_pair(p.first, false));
  }
}

std::unique_ptr<BaseExpr> BaseExpr::create(Graph & graph, YAML::Node yaml)
{
  const auto object = parse_expr_object(yaml);

  if (object.type == "unit") {
    return std::make_unique<UnitExpr>(graph, object.dict);
  }
  if (object.type == "diag") {
    return std::make_unique<DiagExpr>(graph, object.dict);
  }
  if (object.type == "and") {
    return std::make_unique<AndExpr>(graph, object.dict, false);
  }
  if (object.type == "short-circuit-and") {
    return std::make_unique<AndExpr>(graph, object.dict, true);
  }
  if (object.type == "or") {
    return std::make_unique<OrExpr>(graph, object.dict);
  }
  if (object.type == "debug-ok") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::OK);
  }
  if (object.type == "debug-warn") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::WARN);
  }
  if (object.type == "debug-error") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::ERROR);
  }
  if (object.type == "debug-stale") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::STALE);
  }

  throw ConfigError("unknown expr type: " + object.type);
}

ConstExpr::ConstExpr(const DiagnosticLevel level)
{
  level_ = level;
}

ExprStatus ConstExpr::eval() const
{
  ExprStatus status;
  status.level = level_;
  return status;
}

std::vector<BaseNode *> ConstExpr::get_dependency() const
{
  return {};
}

UnitExpr::UnitExpr(Graph & graph, ConfigDict dict)
{
  const auto path = take_expr_text(dict, "path");
  node_ = graph.find_unit(path);
  if (!node_) {
    throw ConfigError("unit node '" + path + "' does not exist");
  }
}

ExprStatus UnitExpr::eval() const
{
  ExprStatus status;
  status.level = node_->level();
  status.links.push_back(std::make_pair(node_, true));
  return status;
}

std::vector<BaseNode *> UnitExpr::get_dependency() const
{
  return {node_};
}

DiagExpr::DiagExpr(Graph & graph, ConfigDict dict)
{
  const auto name = take_expr_text(dict, "name");
  const auto hardware = take_expr_text(dict, "hardware", "");
  node_ = graph.find_diag(name, hardware);
  if (!node_) {
    node_ = graph.make_diag(name, hardware);
  }
}

ExprStatus DiagExpr::eval() const
{
  ExprStatus status;
  status.level = node_->level();
  status.links.push_back(std::make_pair(node_, true));
  return status;
}

std::vector<BaseNode *> DiagExpr::get_dependency() const
{
  return {node_};
}

AndExpr::AndExpr(Graph & graph, ConfigDict dict, bool short_circuit)
{
  short_circuit_ = short_circuit;

  for (const auto & node : take_expr_list(dict, "list")) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

ExprStatus AndExpr::eval() const
{
  if (list_.empty()) {
    ExprStatus status;
    status.level = DiagnosticStatus::OK;
    return status;
  }

  ExprStatus status;
  status.level = DiagnosticStatus::OK;
  for (const auto & expr : list_) {
    const auto result = expr->eval();
    status.level = std::max(status.level, result.level);
    extend(status.links, result.links);
    if (short_circuit_ && status.level != DiagnosticStatus::OK) {
      break;
    }
  }
  status.level = std::min(status.level, DiagnosticStatus::ERROR);
  return status;
}

std::vector<BaseNode *> AndExpr::get_dependency() const
{
  std::vector<BaseNode *> depends;
  for (const auto & expr : list_) {
    const auto nodes = expr->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  return depends;
}

OrExpr::OrExpr(Graph & graph, ConfigDict dict)
{
  for (const auto & node : take_expr_list(dict, "list")) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

ExprStatus OrExpr::eval() const
{
  if (list_.empty()) {
    ExprStatus status;
    status.level = DiagnosticStatus::OK;
    return status;
  }

  ExprStatus status;
  status.level = DiagnosticStatus::ERROR;
  for (const auto & expr : list_) {
    const auto result = expr->eval();
    status.level = std::min(status.level, result.level);
    extend(status.links, result.links);
  }
  status.level = std::min(status.level, DiagnosticStatus::ERROR);
  return status;
}

std::vector<BaseNode *> OrExpr::get_dependency() const
{
  std::vector<BaseNode *> depends;
  for (const auto & expr : list_) {
    const auto nodes = expr->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  return depends;
}

}  // namespace system_diagnostic_graph
