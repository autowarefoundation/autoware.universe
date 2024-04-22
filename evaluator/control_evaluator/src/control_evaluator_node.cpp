// Copyright 2021 Tier IV, Inc.
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

#include "control_evaluator/control_evaluator_node.hpp"

#include "boost/lexical_cast.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_diagnostics
{
controlEvaluatorNode::controlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options)
{
  using std::placeholders::_1;

  control_diag_sub_ = create_subscription<DiagnosticArray>(
    "~/input/diagnostics", 1, std::bind(&controlEvaluatorNode::onDiagnostics, this, _1));

  // List of metrics to calculate and publish
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
}

DiagnosticStatus controlEvaluatorNode::generateDiagnosticStatus() const
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "AEB";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "min";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(0.0);
  status.values.push_back(key_value);
  key_value.key = "max";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(0.5);
  status.values.push_back(key_value);
  key_value.key = "mean";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(1.0);
  status.values.push_back(key_value);
  return status;
}

void controlEvaluatorNode::onDiagnostics(
  [[maybe_unused]] const DiagnosticArray::ConstSharedPtr diag_msg)

{
  const auto start = now();

  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();

  metrics_msg.status.push_back(generateDiagnosticStatus());

  if (!metrics_msg.status.empty()) {
    metrics_pub_->publish(metrics_msg);
  }
  const auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(get_logger(), "control evaluation calculation time: %2.2f ms", runtime * 1e3);
  std::cerr << "control evaluation calculation time: " << runtime << "\n";
}

}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::controlEvaluatorNode)
