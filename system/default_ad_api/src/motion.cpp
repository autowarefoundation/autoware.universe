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

#include "motion.hpp"

#include <memory>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: Node("motion", options), vehicle_stop_checker_(this)
{
  stop_check_duration_ = declare_parameter("stop_check_duration", 1.0);
  enable_starting_state_ = declare_parameter("enable_starting_state", false);

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_srv(srv_accept_, this, &MotionNode::on_accept);
  adaptor.init_pub(pub_state_);
  adaptor.init_cli(cli_set_pause_, group_cli_);
  adaptor.init_sub(sub_is_paused_, this, &MotionNode::on_is_paused);
  adaptor.init_sub(sub_will_move_, this, &MotionNode::on_will_move);

  rclcpp::Rate rate(5);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });

  is_stopping = false;
  is_starting = false;
  state_.state = MotionState::UNKNOWN;
  change_state(MotionState::MOVING);
}

bool MotionNode::call_set_pause(bool pause)
{
  try {
    const auto req = std::make_shared<control_interface::SetPause::Service::Request>();
    req->pause = pause;
    const auto res = cli_set_pause_->call(req);
    return res->status.success;
  } catch (const component_interface_utils::ServiceException &) {
    return false;
  }
}

void MotionNode::change_state(const MotionState::_state_type state)
{
  if (state_.state != state) {
    state_.stamp = now();
    state_.state = state;
    pub_state_->publish(state_);
  }
}

void MotionNode::on_timer()
{
  timer_->cancel();

  if (state_.state == MotionState::MOVING) {
    if (vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
      is_stopping = true;
    }
  }

  if (is_stopping) {
    if (call_set_pause(true)) {
      is_stopping = false;
      change_state(MotionState::STOPPED);
    }
  }

  if (is_starting) {
    if (!vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
      is_starting = false;
      change_state(MotionState::MOVING);
    }
  }

  timer_->reset();
}

void MotionNode::on_is_paused(const control_interface::IsPaused::Message::ConstSharedPtr msg)
{
  (void)msg;
}

void MotionNode::on_will_move(const control_interface::WillMove::Message::ConstSharedPtr msg)
{
  if (msg->data) {
    if (state_.state == MotionState::STOPPED) {
      change_state(MotionState::STARTING);
    }
  } else {
    if (state_.state == MotionState::STARTING) {
      change_state(MotionState::STOPPED);
    }
  }
}

void MotionNode::on_accept(
  const autoware_ad_api::motion::AcceptStart::Service::Request::SharedPtr,
  const autoware_ad_api::motion::AcceptStart::Service::Response::SharedPtr res)
{
  if (state_.state == MotionState::STARTING && !is_starting) {
    if (call_set_pause(false)) {
      is_starting = true;
      res->status.success = true;
    }
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
