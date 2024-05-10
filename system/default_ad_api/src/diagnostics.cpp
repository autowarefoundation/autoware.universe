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

#include "diagnostics.hpp"

#include <memory>

namespace default_ad_api
{

DiagnosticsNode::DiagnosticsNode() : Node("diagnostics")
{
  using std::placeholders::_1;
  pub_struct_ = create_publisher<autoware_adapi_v1_msgs::msg::DiagGraphStruct>(
    "/api/diagnostics/struct", rclcpp::QoS(1));
  pub_status_ = create_publisher<autoware_adapi_v1_msgs::msg::DiagGraphStatus>(
    "/api/diagnostics/status", rclcpp::QoS(1));
  sub_graph_.register_create_callback(std::bind(&DiagnosticsNode::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&DiagnosticsNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 10);
}
void DiagnosticsNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  (void)graph;
}

void DiagnosticsNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  (void)graph;
  /*
  DiagnosticArray array;
  array.header.stamp = graph->updated_stamp();
  for (const auto & unit : graph->units()) {
    if (unit->path().empty()) continue;
    array.status.push_back(unit->create_diagnostic_status());
  }
  pub_array_->publish(array);
  */
}

}  // namespace default_ad_api
