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

#include "dummy_pose_initializer_core.hpp"

DummyPoseInitializer::DummyPoseInitializer() : Node("dummy_pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, BIND_SERVICE(this, OnInitialize));
  pub_reset_ = create_publisher<PoseWithCovarianceStamped>("sim_reset", 1);

  ChangeState(State::Message::UNINITIALIZED);
}

void DummyPoseInitializer::ChangeState(State::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void DummyPoseInitializer::OnInitialize(API_SERVICE_ARG(Initialize, req, res))
{
  try {
    ChangeState(State::Message::INITIALIZING);
    const auto request_pose = req->pose.empty() ? GetGnssPose() : req->pose.front();
    pub_reset_->publish(request_pose);
    res->status.success = true;
    ChangeState(State::Message::INITIALIZED);
  } catch (const component_interface_utils::ServiceException & error) {
    res->status = error.status();
    ChangeState(State::Message::UNINITIALIZED);
  }
}

PoseWithCovarianceStamped DummyPoseInitializer::GetGnssPose()
{
  throw component_interface_utils::ServiceException(
    Initialize::Service::Response::ERROR_GNSS_SUPPORT, "GNSS is not supported.");
}
