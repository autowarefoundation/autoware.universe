// Copyright 2024 The Autoware Contributors
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

namespace diagnostic_graph_utils
{

ConverterNode::ConverterNode() : Node("converter")
{
  using std::placeholders::_1;
  pub_array_ = create_publisher<DiagnosticArray>("/diagnostics_array", rclcpp::QoS(1));
  sub_graph_.register_update_callback(std::bind(&ConverterNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void ConverterNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  const auto & nodes = graph->nodes();
  const auto & diags = graph->diags();

  DiagnosticArray array;
  array.status.reserve(nodes.size() + diags.size());
  for (const auto & node : nodes) {
    const auto & path = node->path();
    if (!path.empty()) {
      DiagnosticStatus msg;
      msg.level = node->level();
      msg.name = path;
      array.status.push_back(msg);
    }
  }
  for (const auto & diag : diags) {
    const auto & path = diag->path();
    if (!path.empty()) {
      DiagnosticStatus msg;
      const auto & status = diag->status();
      msg.level = diag->level();
      msg.name = path;
      msg.message = status.message;
      msg.hardware_id = status.hardware_id;
      msg.values = status.values;
      array.status.push_back(msg);
    }
  }
  pub_array_->publish(array);
}

}  // namespace diagnostic_graph_utils

int main(int argc, char ** argv)
{
  using diagnostic_graph_utils::ConverterNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ConverterNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
