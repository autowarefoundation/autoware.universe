// Copyright 2022 The Autoware Foundation
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

#ifndef DUMMY_NODE_HPP_
#define DUMMY_NODE_HPP_

#include "base_interface/base_interface_node.hpp"

#include <string>
#include <vector>

namespace autoware
{
namespace vehicle
{
namespace interface
{
/**
 * @brief A node that declares all optional interface features
 *
 */
class DummyInterfaceNode : public BaseInterfaceNode
{
public:
  DummyInterfaceNode();

  // Base Interface API
  bool8_t send_control_command(const AckermannControlCommand & msg);
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request);
  void send_gear_command(const GearCommand & msg);
  void send_hand_brake_command(const HandBrakeCommand & msg);
  void send_hazard_lights_command(const HazardLightsCommand & msg);
  void send_headlights_command(const HeadlightsCommand & msg);
  void send_horn_command(const HornCommand & msg);
  void send_wipers_command(const WipersCommand & msg);
  void send_turn_indicators_command(const TurnIndicatorsCommand & msg);

  bool all_received();
  void get_no_receive_names(std::vector<std::string> & result);

  bool cmd_received{false};
  bool mode_change_received{false};
  bool gear_received{false};
  bool hand_brake_received{false};
  bool hazard_lights_received{false};
  bool headlights_received{false};
  bool horn_received{false};
  bool wipers_received{false};
  bool turn_indicators_received{false};
};

/**
 * @brief A node that will throw exception on all optional interface features
 *
 */
class DummyExceptionNode : public BaseInterfaceNode
{
public:
  DummyExceptionNode();
  bool8_t send_control_command(const AckermannControlCommand & msg);
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request);
};
}  // namespace interface
}  // namespace vehicle
}  // namespace autoware

#endif  // DUMMY_NODE_HPP_
