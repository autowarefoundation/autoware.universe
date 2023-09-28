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

BaseNode::BaseNode(const std::string & path) : path_(path)
{
  index_ = 0;
}

UnitNode::UnitNode(const std::string & path) : BaseNode(path)
{
  // for unique_ptr
}

UnitNode::~UnitNode()
{
  // for unique_ptr
}

void UnitNode::create(ConfigObject & config, ExprInit & exprs)
{
  node_.status.level = DiagnosticStatus::STALE;
  node_.status.name = "";
  node_.status.hardware_id = "";

  expr_ = exprs.create(parse_expr_config(config));
}

void UnitNode::update(const rclcpp::Time &)
{
  /*
  const auto result = expr_->eval();
  node_.status.level = result.level;
  node_.links.clear();
  for (const auto & [node, used] : result.links) {
    DiagnosticLink link;
    link.index = node->index();
    link.used = used;
    node_.links.push_back(link);
  }
  */
}

DiagnosticNode UnitNode::report() const
{
  return node_;
}

std::vector<BaseNode *> UnitNode::links() const
{
  return expr_->get_dependency();
}

void DiagNode::create(ConfigObject & config, ExprInit &)
{
  node_.status.level = DiagnosticStatus::STALE;

  timeout_ = 3.0;  // TODO(Takagi, Isamu): parameterize
  name_ = config.take_text("name");
  hardware_ = config.take_text("hardware", "");
}

std::pair<std::string, std::string> DiagNode::key() const
{
  return std::make_pair(name_, hardware_);
}

void DiagNode::update(const rclcpp::Time & stamp)
{
  if (time_) {
    const auto elapsed = (stamp - time_.value()).seconds();
    if (timeout_ < elapsed) {
      const auto name = node_.status.name;
      const auto hardware = node_.status.hardware_id;
      node_.status = DiagnosticStatus();
      node_.status.level = DiagnosticStatus::STALE;
      node_.status.name = name;
      node_.status.hardware_id = hardware;
      time_ = std::nullopt;
    }
  }
}

DiagnosticNode DiagNode::report() const
{
  return node_;
}

void DiagNode::callback(const DiagnosticStatus & status, const rclcpp::Time & stamp)
{
  node_.status = status;
  time_ = stamp;
}

}  // namespace system_diagnostic_graph
