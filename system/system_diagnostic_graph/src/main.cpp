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

#include "main.hpp"

#include <memory>

namespace system_diagnostic_graph
{

MainNode::MainNode() : Node("system_diagnostic_graph")
{
  // Init ros interface.
  {
    using std::placeholders::_1;
    const auto sub_qos = rclcpp::QoS(declare_parameter<int64_t>("array_qos_depth"));
    const auto pub_qos = rclcpp::QoS(declare_parameter<int64_t>("graph_qos_depth"));
    sub_array_ = create_subscription<DiagnosticArray>(
      "/diagnostics", sub_qos, std::bind(&MainNode::on_diag, this, _1));
    pub_graph_ = create_publisher<DiagnosticGraph>("/diagnostics_graph", pub_qos);

    const auto rate = rclcpp::Rate(declare_parameter<int64_t>("rate"));
    timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  }
}

void MainNode::on_timer()
{
  graph_.debug();

  const auto report = graph_.report(now());
  pub_graph_->publish(report);
}

void MainNode::on_diag(const DiagnosticArray::ConstSharedPtr msg)
{
  graph_.update(*msg);
}

}  // namespace system_diagnostic_graph

int main(int argc, char ** argv)
{
  using system_diagnostic_graph::MainNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MainNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
