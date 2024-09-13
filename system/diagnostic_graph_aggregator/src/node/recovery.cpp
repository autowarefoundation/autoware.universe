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

#include "recovery.hpp"

#include <algorithm>

namespace diagnostic_graph_aggregator
{

RecoveryNode::RecoveryNode() : Node("recovery")
{
  using std::placeholders::_1;
  const auto qos_graph = rclcpp::QoS(1);
  const auto qos_aw_state = rclcpp::QoS(1);
  const auto qos_mrm_state = rclcpp::QoS(1);

  const auto callback_graph = std::bind(&RecoveryNode::on_graph, this, _1);
  sub_graph_ =
    create_subscription<DiagnosticGraph>("/diagnostics_graph", qos_graph, callback_graph);
  const auto callback_aw_state = std::bind(&RecoveryNode::on_aw_state, this, _1);
  sub_aw_state_ =
    create_subscription<AutowareState>("/autoware/state", qos_aw_state, callback_aw_state);
  const auto callback_mrm_state = std::bind(&RecoveryNode::on_mrm_state, this, _1);
  sub_mrm_state_ =
    create_subscription<MrmState>("/system/fail_safe/mrm_state", qos_mrm_state, callback_mrm_state);
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_clear_mrm_ = create_client<std_srvs::srv::Trigger>(
    "/system/clear_mrm", rmw_qos_profile_services_default, callback_group_);

  fatal_error_ = false;
  mrm_occur_ = false;
  autonomous_available_ = false;
  mrm_by_fatal_error_ = false;
}

void RecoveryNode::on_graph(const DiagnosticGraph::ConstSharedPtr msg)
{
  for (const auto & node : msg->nodes) {
    if (node.status.name == "/autoware/modes/autonomous") {
      autonomous_available_ = node.status.level == DiagnosticStatus::OK;
    }
    // aggregate non-recoverable error
    if (node.status.name == "/autoware/fatal_error/autonomous_available") {
      if (node.status.level != DiagnosticStatus::OK) {
        fatal_error_ = true;
      } else {
        fatal_error_ = false;
      }
    }
  }
}

void RecoveryNode::on_aw_state(const AutowareState::ConstSharedPtr msg)
{
  auto_driving_ = msg->state == AutowareState::DRIVING;
}

void RecoveryNode::on_mrm_state(const MrmState::ConstSharedPtr msg)
{
  // set flag if mrm happened by fatal error
  if (msg->state != MrmState::NORMAL && fatal_error_) {
    mrm_by_fatal_error_ = true;
  }
  // reset flag if recoverd (transition from mrm to normal)
  if (mrm_occur_ && msg->state == MrmState::NORMAL) {
    mrm_by_fatal_error_ = false;
  }
  mrm_occur_ = msg->state != MrmState::NORMAL;
  // 1. Not emergency
  // 2. Non-recovoerable MRM have not happened
  // 3. on MRM
  // 4. on autonomous driving
  if (autonomous_available_ && !mrm_by_fatal_error_ && mrm_occur_ && auto_driving_) {
    clear_mrm();
  }
}

void RecoveryNode::clear_mrm()
{
  const auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto logger = get_logger();
  if (!srv_clear_mrm_->service_is_ready()) {
    RCLCPP_ERROR(logger, "MRM clear server is not ready.");
    return;
  }
  RCLCPP_INFO(logger, "Recover MRM automatically.");
  auto res = srv_clear_mrm_->async_send_request(req);
  std::future_status status = res.wait_for(std::chrono::milliseconds(50));
  if (status == std::future_status::timeout) {
    RCLCPP_INFO(logger, "Service timeout");
    return;
  }
  if (!res.get()->success) {
    RCLCPP_INFO(logger, "Recovering MRM failed.");
    return;
  }
  RCLCPP_INFO(logger, "Recovering MRM succeed.");
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  using diagnostic_graph_aggregator::RecoveryNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<RecoveryNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
