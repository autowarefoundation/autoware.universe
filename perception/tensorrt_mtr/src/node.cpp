// Copyright 2024 TIER IV, Inc.
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

#include "tensorrt_mtr/node.hpp"

namespace trt_mtr
{
MTRNode::MTRNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_mtr_node", node_options)
{
  // TODO(ktro2828)
}

void MTRNode::callback(const TrackedObjects::ConstSharedPtr object_msg)
{
  // TODO(ktro2828)
}

void MTRNode::onMap(const HADMapBin::ConstSharedPtr map_msg)
{
  // TODO(ktro2828)
}

void convertLaneletToPolyline()
{
  // TODO(ktro2828)
}

void updateAgentHistory(const float current_time)
{
  // TODO(ktro2828)
}

}  // namespace trt_mtr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trt_mtr::MTRNode);
