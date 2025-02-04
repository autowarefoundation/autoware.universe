// Copyright 2023 LeoDrive, A.Ş.
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

#ifndef MODERATE_STOP_INTERFACE_HPP_
#define MODERATE_STOP_INTERFACE_HPP_

#include <autoware/component_interface_specs_universe/control.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

namespace autoware::vehicle_cmd_gate
{

class ModerateStopInterface
{
private:
  using SetStop = autoware::component_interface_specs_universe::control::SetStop;
  using IsStopped = autoware::component_interface_specs_universe::control::IsStopped;
  using IsStartRequested = autoware::component_interface_specs_universe::control::IsStartRequested;

public:
  explicit ModerateStopInterface(rclcpp::Node * node);
  bool is_stop_requested() const;
  void publish();

private:
  IsStopped::Message stop_state_;
  std::unordered_map<std::string, bool> stop_map_;
  std::optional<std::unordered_map<std::string, bool>> prev_stop_map_;

  rclcpp::Node * node_;
  autoware::component_interface_utils::Service<SetStop>::SharedPtr srv_set_stop_;
  autoware::component_interface_utils::Publisher<IsStopped>::SharedPtr pub_is_stopped_;

  void on_stop_request(
    const SetStop::Service::Request::SharedPtr req,
    const SetStop::Service::Response::SharedPtr res);

  void update_stop_state();
};

}  // namespace autoware::vehicle_cmd_gate

#endif  // MODERATE_STOP_INTERFACE_HPP_
