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

#ifndef NODE__RECOVERY_HPP_
#define NODE__RECOVERY_HPP_

#include "graph/types.hpp"

#include <rclcpp/rclcpp.hpp>

// Autoware
#include <component_interface_utils/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <functional>
#include <map>  // Use map for sorting keys.
#include <memory>
#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

class RecoveryNode : public rclcpp::Node
{
public:
  RecoveryNode();

private:
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using MrmState = autoware_adapi_v1_msgs::msg::MrmState;

  bool fatal_error_;
  bool autonomous_available_;
  bool mrm_occur_;
  bool auto_driving_;
  bool mrm_by_fatal_error_;
  rclcpp::Subscription<DiagnosticGraph>::SharedPtr sub_graph_;
  rclcpp::Subscription<AutowareState>::SharedPtr sub_aw_state_;
  rclcpp::Subscription<MrmState>::SharedPtr sub_mrm_state_;

  // service
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv_clear_mrm_;

  // callback group for service
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void on_graph(const DiagnosticGraph::ConstSharedPtr msg);
  void on_aw_state(const AutowareState::ConstSharedPtr msg);
  void on_mrm_state(const MrmState::ConstSharedPtr msg);

  void clear_mrm();
};

}  // namespace diagnostic_graph_aggregator

#endif  // NODE__RECOVERY_HPP_
