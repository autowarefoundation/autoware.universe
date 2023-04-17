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

#include "pause_interface.hpp"

namespace vehicle_cmd_gate
{

PauseInterface::PauseInterface(rclcpp::Node * node) : node_(node)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(node);
  adaptor.init_srv(srv_set_pause_, this, &PauseInterface::on_pause);
  adaptor.init_pub(pub_is_paused_);
  adaptor.init_pub(pub_is_start_requested_);

  pause_state_.data = false;
  pause_state_.requested_sources.clear();
  is_start_requested_ = false;
  publish();
}

bool PauseInterface::is_paused()
{
  return pause_state_.data;
}

void PauseInterface::publish()
{
  if (prev_pause_map_ != pause_map_) {
    update_pause_state();
    pub_is_paused_->publish(pause_state_);
    prev_pause_map_ = pause_map_;
  }

  if (prev_is_start_requested_ != is_start_requested_) {
    IsStartRequested::Message msg;
    msg.stamp = node_->now();
    msg.data = is_start_requested_;
    pub_is_start_requested_->publish(msg);
    prev_is_start_requested_ = is_start_requested_;
  }
}

void PauseInterface::update(const AckermannControlCommand & control)
{
  is_start_requested_ = eps < std::abs(control.longitudinal.speed);
}

void PauseInterface::on_pause(
  const SetPause::Service::Request::SharedPtr req, const SetPause::Service::Response::SharedPtr res)
{
  pause_map_[req->request_source] = req->pause;
  res->status.success = true;
}

void PauseInterface::update_pause_state()
{
  pause_state_.stamp = node_->now();
  pause_state_.requested_sources.clear();
  pause_state_.data = false;

  if (pause_map_.empty()) {
    return;
  }
  for (auto & itr : pause_map_) {
    if (itr.second) {
      pause_state_.data = true;
      pause_state_.requested_sources.push_back(itr.first);
    }
  }
}
}  // namespace vehicle_cmd_gate
