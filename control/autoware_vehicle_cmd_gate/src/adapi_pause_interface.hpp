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

#ifndef ADAPI_PAUSE_INTERFACE_HPP_
#define ADAPI_PAUSE_INTERFACE_HPP_

#include <autoware/component_interface_specs_universe/control.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>

namespace autoware::vehicle_cmd_gate
{

class AdapiPauseInterface
{
private:
  static constexpr double eps = 1e-3;
  using Control = autoware_control_msgs::msg::Control;
  using SetPause = autoware::component_interface_specs_universe::control::SetPause;
  using IsPaused = autoware::component_interface_specs_universe::control::IsPaused;
  using IsStartRequested = autoware::component_interface_specs_universe::control::IsStartRequested;

public:
  explicit AdapiPauseInterface(rclcpp::Node * node);
  bool is_paused() const;
  void publish();
  void update(const Control & control);

private:
  bool is_paused_;
  bool is_start_requested_;
  std::optional<bool> prev_is_paused_;
  std::optional<bool> prev_is_start_requested_;

  rclcpp::Node * node_;
  autoware::component_interface_utils::Service<SetPause>::SharedPtr srv_set_pause_;
  autoware::component_interface_utils::Publisher<IsPaused>::SharedPtr pub_is_paused_;
  autoware::component_interface_utils::Publisher<IsStartRequested>::SharedPtr
    pub_is_start_requested_;

  void on_pause(
    const SetPause::Service::Request::SharedPtr req,
    const SetPause::Service::Response::SharedPtr res);
};

}  // namespace autoware::vehicle_cmd_gate

#endif  // ADAPI_PAUSE_INTERFACE_HPP_
