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

#include "converter.hpp"

#include <utility>
#include <vector>

namespace
{

using autoware_auto_system_msgs::msg::HazardStatus;
using autoware_auto_system_msgs::msg::HazardStatusStamped;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_system_msgs::msg::DiagnosticGraph;
using tier4_system_msgs::msg::DiagnosticNode;
using DiagnosticLevel = DiagnosticStatus::_level_type;

enum class HazardLevel { NF, SF, LF, SPF };

struct TempNode
{
  const DiagnosticNode & node;
  bool is_auto_tree;
};

HazardLevel get_hazard_level(const TempNode & node, DiagnosticLevel auto_mode_level)
{
  // Convert the level according to the table below.
  // The Level other than auto mode is considered OK.
  // |-------|-------------------------------|
  // | Level |           Root level          |
  // |-------|-------------------------------|
  // |       | OK    | WARN  | ERROR | STALE |
  // | OK    | NF    | NF    | NF    | NF    |
  // | WARN  | SF    | LF    | LF    | LF    |
  // | ERROR | SF    | LF    | SPF   | SPF   |
  // | STALE | SF    | LF    | SPF   | SPF   |
  // |-------|-------------------------------|

  const auto root_level = node.is_auto_tree ? auto_mode_level : DiagnosticStatus::OK;
  const auto node_level = node.node.status.level;

  if (node_level == DiagnosticStatus::OK) {
    return HazardLevel::NF;
  }
  if (root_level == DiagnosticStatus::OK) {
    return HazardLevel::SF;
  }
  if (node_level == DiagnosticStatus::WARN) {
    return HazardLevel::LF;
  }
  if (root_level == DiagnosticStatus::WARN) {
    return HazardLevel::LF;
  }
  return HazardLevel::SPF;
}

void set_auto_tree(std::vector<TempNode> & nodes, int index)
{
  TempNode & node = nodes[index];
  if (node.is_auto_tree) {
    return;
  }

  node.is_auto_tree = true;
  for (const auto & link : node.node.links) {
    set_auto_tree(nodes, link.index);
  }
}

void merge_hazard_status(
  HazardStatusStamped & hazard, const std::string prefix, const DiagnosticGraph & graph)
{
  std::vector<TempNode> nodes;
  nodes.reserve(graph.nodes.size());
  for (const auto & node : graph.nodes) {
    nodes.push_back({node, false});
  }

  DiagnosticLevel auto_mode_level = DiagnosticStatus::STALE;
  for (size_t index = 0; index < nodes.size(); ++index) {
    const auto & status = nodes[index].node.status;
    if (status.name == "/autoware/modes/autonomous") {
      set_auto_tree(nodes, index);
      auto_mode_level = status.level;
    }
  }

  for (const auto & node : nodes) {
    switch (get_hazard_level(node, auto_mode_level)) {
      case HazardLevel::NF:
        hazard.status.diag_no_fault.push_back(node.node.status);
        hazard.status.diag_no_fault.back().name = prefix + node.node.status.name;
        break;
      case HazardLevel::SF:
        hazard.status.diag_safe_fault.push_back(node.node.status);
        hazard.status.diag_safe_fault.back().name = prefix + node.node.status.name;
        break;
      case HazardLevel::LF:
        hazard.status.diag_latent_fault.push_back(node.node.status);
        hazard.status.diag_latent_fault.back().name = prefix + node.node.status.name;
        break;
      case HazardLevel::SPF:
        hazard.status.diag_single_point_fault.push_back(node.node.status);
        hazard.status.diag_single_point_fault.back().name = prefix + node.node.status.name;
        break;
    }
  }
}

}  // namespace

namespace hazard_status_converter
{

Converter::Converter(const rclcpp::NodeOptions & options) : Node("converter", options)
{
  const auto inputs = std::vector<std::pair<std::string, std::string>>(
    {{"/mm", "/main/diagnostics_graph/main"},
     {"/ss", "/sub/diagnostics_graph/sub"},
     {"/vm", "/supervisor/diagnostics_graph/main"},
     {"/vs", "/supervisor/diagnostics_graph/sub"},
     {"/vv", "/supervisor/diagnostics_graph/supervisor"}});

  for (const auto & [prefix, topic] : inputs) {
    const std::function<void(const DiagnosticGraph::ConstSharedPtr)> callback =
      std::bind(&Converter::on_graph, this, prefix, std::placeholders::_1);
    sub_graphs_.push_back(create_subscription<DiagnosticGraph>(topic, rclcpp::QoS(1), callback));
  }
  pub_hazard_ = create_publisher<HazardStatusStamped>("/hazard_status", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void Converter::on_graph(const std::string & prefix, const DiagnosticGraph::ConstSharedPtr msg)
{
  graphs_[prefix] = msg;
}

void Converter::on_timer()
{
  const auto get_level = [](const HazardStatus & status) {
    if (!status.diag_single_point_fault.empty()) {
      return HazardStatus::SINGLE_POINT_FAULT;
    }
    if (!status.diag_latent_fault.empty()) {
      return HazardStatus::LATENT_FAULT;
    }
    if (!status.diag_safe_fault.empty()) {
      return HazardStatus::SAFE_FAULT;
    }
    return HazardStatus::NO_FAULT;
  };

  HazardStatusStamped hazard;
  hazard.stamp = now();
  for (const auto & [prefix, graph] : graphs_) {
    merge_hazard_status(hazard, prefix, *graph);
  }
  hazard.status.level = get_level(hazard.status);
  hazard.status.emergency = hazard.status.level == HazardStatus::SINGLE_POINT_FAULT;
  hazard.status.emergency_holding = false;
  pub_hazard_->publish(hazard);
}

}  // namespace hazard_status_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hazard_status_converter::Converter)
