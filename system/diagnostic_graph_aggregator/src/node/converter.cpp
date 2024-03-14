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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

std::string level_to_string(DiagnosticLevel level)
{
  switch (level) {
    case DiagnosticStatus::OK:
      return "OK";
    case DiagnosticStatus::WARN:
      return "WARN";
    case DiagnosticStatus::ERROR:
      return "ERROR";
    case DiagnosticStatus::STALE:
      return "STALE";
  }
  return "UNKNOWN";
}

std::string parent_path(const std::string & path)
{
  return path.substr(0, path.rfind('/'));
}

std::vector<std::string> complement_paths(const DiagnosticGraph & graph)
{
  std::unordered_map<std::string, bool> paths;
  for (const auto & node : graph.nodes) {
    std::string path = node.status.name;
    paths[path] = false;
    while (path = parent_path(path), !path.empty()) {
      if (paths.count(path)) break;
      paths[path] = true;
    }
  }
  std::vector<std::string> result;
  for (const auto & [path, flag] : paths) {
    if (flag) result.push_back(path);
  }
  return result;
}

ConverterNode::ConverterNode() : Node("converter")
{
  using std::placeholders::_1;
  const auto qos_graph = rclcpp::QoS(1);
  const auto qos_array = rclcpp::QoS(1);

  const auto callback = std::bind(&ConverterNode::on_graph, this, _1);
  sub_graph_ = create_subscription<DiagnosticGraph>("/diagnostics_graph", qos_graph, callback);
  pub_array_ = create_publisher<DiagnosticArray>("/diagnostics_agg", qos_array);
  complement_inner_nodes_ = declare_parameter<bool>("complement_inner_nodes");
}

void ConverterNode::on_graph(const DiagnosticGraph::ConstSharedPtr msg)
{
  DiagnosticArray message;
  message.header.stamp = msg->stamp;
  message.status.reserve(msg->nodes.size());
  for (const auto & node : msg->nodes) {
    message.status.push_back(node.status);
    for (const auto & link : node.links) {
      diagnostic_msgs::msg::KeyValue kv;
      const auto & status = msg->nodes[link.index].status;
      kv.key = status.name;
      kv.value = level_to_string(status.level);
      if (link.used) {
        message.status.back().values.push_back(kv);
      }
    }
  }

  if (complement_inner_nodes_) {
    if (!inner_node_names_) {
      inner_node_names_ = complement_paths(*msg);
    }
    for (const auto & name : inner_node_names_.value()) {
      message.status.emplace_back();
      message.status.back().name = name;
      message.status.back().level = DiagnosticStatus::STALE;
    }
  }

  pub_array_->publish(message);
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  using diagnostic_graph_aggregator::ConverterNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ConverterNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
