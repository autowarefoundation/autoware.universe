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

#include "expr.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <utility>

namespace system_diagnostic_graph
{

UnitNode::UnitNode(const std::string & name) : name_(name)
{
  level_ = DiagnosticStatus::STALE;
}

UnitNode::~UnitNode()
{
  // for unique_ptr
}

DiagnosticNode UnitNode::report() const
{
  DiagnosticStatus status;
  status.level = level_;
  status.name = name_;
  status.hardware_id = "";

  DiagnosticNode message;
  message.status = status;
  return message;
}

void UnitNode::update()
{
  const auto result = expr_->eval();
  level_ = result.level;
}

void UnitNode::create(Graph & graph, const NodeConfig & config)
{
  try {
    expr_ = BaseExpr::create(graph, config->yaml);
  } catch (const ConfigError & error) {
    throw create_error(config, error.what());
  }
}

std::vector<BaseNode *> UnitNode::links() const
{
  return expr_->get_dependency();
}

DiagNode::DiagNode(const std::string & name, const std::string & hardware)
: name_(name), hardware_(hardware)
{
  level_ = DiagnosticStatus::STALE;
}

DiagnosticNode DiagNode::report() const
{
  DiagnosticStatus status = status_;
  status.level = level_;
  status.name = name_;
  status.hardware_id = hardware_;

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

}  // namespace system_diagnostic_graph
