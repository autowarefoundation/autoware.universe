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

#ifndef MERGER_HPP_
#define MERGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/diagnostic_graph.hpp>

#include <vector>

namespace hazard_status_converter
{

class Merger : public rclcpp::Node
{
public:
  explicit Merger(const rclcpp::NodeOptions & options);

private:
  using DiagnosticGraph = tier4_system_msgs::msg::DiagnosticGraph;
  std::vector<rclcpp::Subscription<DiagnosticGraph>::SharedPtr> sub_inputs_;
  rclcpp::Publisher<DiagnosticGraph>::SharedPtr pub_output_;
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer();
  void on_graph(const DiagnosticGraph::ConstSharedPtr msg, int index);
};

}  // namespace hazard_status_converter

#endif  // MERGER_HPP_
