// Copyright 2020 Autoware Foundation
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

#include "automatic_api_caller.hpp"

#include <memory>

AutomaticApiCaller::AutomaticApiCaller() : Node("automatic_api_caller")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_cli(cli_initialize_, group_cli_);
  node.init_sub(sub_state_, [this](API_MESSAGE_ARG(State, msg)) { state_ = *msg; });

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { OnTimer(); });

  state_.stamp = now();
  state_.state = State::Message::UNKNOWN;
}

void AutomaticApiCaller::OnTimer()
{
  if (state_.state == State::Message::UNINITIALIZED) {
    RCLCPP_INFO_STREAM(get_logger(), "Request to initialize pose.");
    const auto req = std::make_shared<Initialize::Service::Request>();
    try {
      cli_initialize_->call(req);
    } catch (const component_interface_utils::ServiceException & error) {
    }
  }
}
