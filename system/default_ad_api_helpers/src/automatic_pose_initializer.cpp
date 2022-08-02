// Copyright 2022 Autoware Foundation
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

#include "automatic_pose_initializer.hpp"

#include <memory>

namespace default_ad_api_helpers
{

AutomaticPoseInitializer::AutomaticPoseInitializer() : Node("automatic_pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_cli(cli_initialize_, group_cli_);
  node.init_sub(sub_state_, [this](const State::Message::ConstSharedPtr msg) { state_ = *msg; });

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { OnTimer(); });

  state_.stamp = now();
  state_.state = State::Message::UNKNOWN;
}

void AutomaticPoseInitializer::OnTimer()
{
  if (state_.state == State::Message::UNINITIALIZED) {
    try {
      const auto req = std::make_shared<Initialize::Service::Request>();
      cli_initialize_->call(req);
    } catch (const component_interface_utils::ServiceException & error) {
    }
  }
}

}  // namespace default_ad_api_helpers

#include "macros/create_node.hpp"
CREATE_MULTI_THREAD_NODE(default_ad_api_helpers::AutomaticPoseInitializer)
