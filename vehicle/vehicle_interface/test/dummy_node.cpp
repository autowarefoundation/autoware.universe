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

#include "dummy_node.hpp"
#include <vector>
#include <string>

namespace autoware
{
namespace vehicle
{
namespace interface
{
DummyInterfaceNode::DummyInterfaceNode()
: VehicleInterfaceNode("dummy_interface", {InterfaceFeature::GEAR, InterfaceFeature::HAND_BRAKE,
      InterfaceFeature::HAZARD_LIGHTS, InterfaceFeature::HEADLIGHTS,
      InterfaceFeature::HORN, InterfaceFeature::ODOMETRY,
      InterfaceFeature::TURN_INDICATORS, InterfaceFeature::WIPERS},
    rclcpp::NodeOptions{})
{
}

bool8_t DummyInterfaceNode::send_control_command(const AckermannControlCommand & msg)
{
  (void)msg;
  cmd_received = true;
  return true;
}
bool8_t DummyInterfaceNode::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  (void)request;
  mode_change_received = true;
  return true;
}

void DummyInterfaceNode::send_gear_command(const GearCommand & msg)
{
  (void)msg;
  gear_received = true;
}

void DummyInterfaceNode::send_hand_brake_command(const HandBrakeCommand & msg)
{
  (void)msg;
  hand_brake_received = true;
}

void DummyInterfaceNode::send_hazard_lights_command(const HazardLightsCommand & msg)
{
  (void)msg;
  hazard_lights_received = true;
}

void DummyInterfaceNode::send_headlights_command(const HeadlightsCommand & msg)
{
  (void)msg;
  headlights_received = true;
}

void DummyInterfaceNode::send_horn_command(const HornCommand & msg)
{
  (void)msg;
  horn_received = true;
}

void DummyInterfaceNode::send_wipers_command(const WipersCommand & msg)
{
  (void)msg;
  wipers_received = true;
}

void DummyInterfaceNode::send_turn_indicators_command(const TurnIndicatorsCommand & msg)
{
  (void)msg;
  turn_indicators_received = true;
}

bool DummyInterfaceNode::all_received()
{
  return cmd_received && mode_change_received && gear_received && hand_brake_received &&
         hazard_lights_received && headlights_received && horn_received && wipers_received &&
         turn_indicators_received;
}

void DummyInterfaceNode::get_no_receive_names(std::vector<std::string> & result)
{
  if (!cmd_received) {
    result.push_back("AckermannControlCommand");
  }
  if (!mode_change_received) {
    result.push_back("ModeChangeRequest");
  }
  if (!gear_received) {
    result.push_back("GearCommand");
  }
  if (!hand_brake_received) {
    result.push_back("HandBrakeCommand");
  }
  if (!hazard_lights_received) {
    result.push_back("HazardLightsCommand");
  }
  if (!headlights_received) {
    result.push_back("HeadlightsCommand");
  }
  if (!horn_received) {
    result.push_back("HornCommand");
  }
  if (!wipers_received) {
    result.push_back("WipersCommand");
  }
  if (!turn_indicators_received) {
    result.push_back("TurnIndicatorsCommand");
  }
}

DummyExceptionNode::DummyExceptionNode()
: VehicleInterfaceNode("dummy_exception_node", {}, rclcpp::NodeOptions{})
{
}
bool8_t DummyExceptionNode::send_control_command(const AckermannControlCommand & msg)
{
  (void) msg;
  return false;
}
bool8_t DummyExceptionNode::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  (void) request;
  return false;
}
}  // namespace interface
}  // namespace vehicle
}  // namespace autoware
