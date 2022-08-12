// Copyright 2022 TIER IV, Inc.
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

#include "operation_mode.hpp"

#include <memory>

namespace default_ad_api
{

using OperationModeRequest = system_interface::ChangeOperationMode::Service::Request;
using OperationMode = OperationModeRequest::_operation_type;
using AutowareControlRequest = system_interface::ChangeAutowareControl::Service::Request;

OperationModeNode::OperationModeNode(const rclcpp::NodeOptions & options) : Node("routing", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_sub(sub_state_, this, &OperationModeNode::on_state);
  adaptor.init_pub(pub_state_);
  adaptor.init_srv(srv_stop_mode_, this, &OperationModeNode::on_change_to_stop);
  adaptor.init_srv(srv_autonomous_mode_, this, &OperationModeNode::on_change_to_autonomous);
  adaptor.init_srv(srv_local_mode_, this, &OperationModeNode::on_change_to_local);
  adaptor.init_srv(srv_remote_mode_, this, &OperationModeNode::on_change_to_remote);
  adaptor.init_srv(srv_enable_control_, this, &OperationModeNode::on_enable_autoware_control);
  adaptor.init_srv(srv_disable_control_, this, &OperationModeNode::on_disable_autoware_control);
  adaptor.init_cli(cli_mode_, group_cli_);
  adaptor.init_cli(cli_control_, group_cli_);
}

void OperationModeNode::on_change_to_stop(
  const ChangeToStop::Service::Request::SharedPtr,
  const ChangeToStop::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<OperationModeRequest>();
  req->operation.mode = OperationMode::STOP;
  res->status = cli_mode_->call(req)->status;
}

void OperationModeNode::on_change_to_autonomous(
  const ChangeToAutonomous::Service::Request::SharedPtr,
  const ChangeToAutonomous::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<OperationModeRequest>();
  req->operation.mode = OperationMode::AUTONOMOUS;
  res->status = cli_mode_->call(req)->status;
}

void OperationModeNode::on_change_to_local(
  const ChangeToLocal::Service::Request::SharedPtr,
  const ChangeToLocal::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<OperationModeRequest>();
  req->operation.mode = OperationMode::LOCAL;
  res->status = cli_mode_->call(req)->status;
}

void OperationModeNode::on_change_to_remote(
  const ChangeToRemote::Service::Request::SharedPtr,
  const ChangeToRemote::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<OperationModeRequest>();
  req->operation.mode = OperationMode::REMOTE;
  res->status = cli_mode_->call(req)->status;
}

void OperationModeNode::on_enable_autoware_control(
  const EnableAutowareControl::Service::Request::SharedPtr,
  const EnableAutowareControl::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<AutowareControlRequest>();
  req->autoware_control = true;
  res->status = cli_control_->call(req)->status;
}

void OperationModeNode::on_disable_autoware_control(
  const DisableAutowareControl::Service::Request::SharedPtr,
  const DisableAutowareControl::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<AutowareControlRequest>();
  req->autoware_control = false;
  res->status = cli_control_->call(req)->status;
}

void OperationModeNode::on_state(const OperationModeState::Message::ConstSharedPtr msg)
{
  pub_state_->publish(*msg);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::OperationModeNode)
