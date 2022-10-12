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
#include <unordered_map>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: Node("motion", options), vehicle_stop_checker_(this)
{
  stop_check_duration_ = declare_parameter("stop_check_duration", 1.0);
  enable_starting_state_ = declare_parameter("enable_starting_state", false);
  waiting_for_set_pause_ = false;

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_srv(srv_accept_, this, &MotionNode::on_accept);
  adaptor.init_pub(pub_state_);
  adaptor.init_cli(cli_set_pause_, group_cli_);
  adaptor.init_sub(sub_is_paused_, this, &MotionNode::on_is_paused);
  adaptor.init_sub(sub_is_start_requested_, this, &MotionNode::on_is_start_requested);

  rclcpp::Rate rate(5);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  change_state(State::kMoving, true);
}

void MotionNode::change_state(const State state, const bool init)
{
  using MotionState = autoware_ad_api::motion::State::Message;
  static const auto mapping = std::unordered_map<State, MotionState::_state_type>(
    {{State::kMoving, MotionState::MOVING},
     {State::kStopping, MotionState::STOPPED},
     {State::kStopped, MotionState::STOPPED},
     {State::kStarting, MotionState::STARTING},
     {State::kStarted, MotionState::STARTING}});

  if (init || mapping.at(state_) != mapping.at(state)) {
    MotionState msg;
    msg.stamp = now();
    msg.state = mapping.at(state);
    pub_state_->publish(msg);
  }
  state_ = state;
}

void MotionNode::on_timer()
{
  if (state_ == State::kMoving) {
    if (vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
      change_state(State::kStopping);
    }
  }

  if (state_ == State::kStarted) {
    if (!vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
      change_state(State::kMoving);
    }
  }

  if (state_ == State::kStopping) {
    if (!waiting_for_set_pause_ && cli_set_pause_->get_rclcpp_client()->service_is_ready()) {
      const auto req = std::make_shared<control_interface::SetPause::Service::Request>();
      req->pause = true;
      waiting_for_set_pause_ = true;
      cli_set_pause_->async_send_request(req, [this](auto) { waiting_for_set_pause_ = false; });
    }
  }
}

void MotionNode::on_is_paused(const control_interface::IsPaused::Message::ConstSharedPtr msg)
{
  switch (state_) {
    case State::kMoving:
    case State::kStopping:
    case State::kStarted:
      if (msg->data) {
        change_state(State::kStopped);
      }
      break;
    case State::kStopped:
    case State::kStarting:
      if (!msg->data) {
        change_state(State::kStarted);
      }
      break;
  }
}

void MotionNode::on_is_start_requested(
  const control_interface::IsStartRequested::Message::ConstSharedPtr msg)
{
  if (msg->data) {
    if (state_ == State::kStopped) {
      return change_state(State::kStarting);
    }
  } else {
    if (state_ == State::kStarting) {
      return change_state(State::kStopped);
    }
    if (state_ == State::kStarted) {
      return change_state(State::kStopping);
    }
  }
}

void MotionNode::on_accept(
  const autoware_ad_api::motion::AcceptStart::Service::Request::SharedPtr,
  const autoware_ad_api::motion::AcceptStart::Service::Response::SharedPtr res)
{
  if (state_ != State::kStarting) {
    using AcceptStartResponse = autoware_ad_api::motion::AcceptStart::Service::Response;
    throw component_interface_utils::ServiceException(
      AcceptStartResponse::ERROR_NOT_STARTING, "The motion state is not starting");
  }

  const auto inner_req = std::make_shared<control_interface::SetPause::Service::Request>();
  inner_req->pause = false;

  const auto inner_res = cli_set_pause_->call(inner_req);
  component_interface_utils::status::copy(inner_res, res);  // NOLINT
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
