// Copyright 2020-2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief Base class for vehicle "translator"
#ifndef VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
#define VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_

#include <common/types.hpp>
#include <autoware_auto_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_msgs/msg/headlights_command.hpp>
#include <autoware_auto_msgs/msg/headlights_report.hpp>
#include <autoware_auto_msgs/msg/horn_command.hpp>
#include <autoware_auto_msgs/msg/horn_report.hpp>
#include <autoware_auto_msgs/msg/wipers_command.hpp>
#include <autoware_auto_msgs/msg/wipers_report.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <chrono>

using autoware::common::types::bool8_t;

using autoware_auto_msgs::msg::HazardLightsCommand;
using autoware_auto_msgs::msg::HazardLightsReport;
using autoware_auto_msgs::msg::HeadlightsCommand;
using autoware_auto_msgs::msg::HeadlightsReport;
using autoware_auto_msgs::msg::HornCommand;
using autoware_auto_msgs::msg::HornReport;
using autoware_auto_msgs::msg::WipersCommand;
using autoware_auto_msgs::msg::WipersReport;
using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::AckermannControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::msg::VehicleStateReport;
using autoware_auto_msgs::msg::VehicleOdometry;
using ModeChangeRequest = autoware_auto_msgs::srv::AutonomyModeChange_Request;
using ModeChangeResponse = autoware_auto_msgs::srv::AutonomyModeChange_Response;

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

/// Interface class for specific vehicle implementations. Child classes which implement this
/// interface are expected to have wrap their own communication mechanism, and create a subclass
/// from the VehicleInterfaceNode
class VEHICLE_INTERFACE_PUBLIC PlatformInterface
{
public:
  /// Constructor
  PlatformInterface() = default;
  /// Destructor
  virtual ~PlatformInterface() = default;

  /// Try to receive data from the vehicle platform, and update StateReport and Odometry. If no
  /// data is received, return false, and the VehicleInterfaceNode will treat this as an error
  /// If the platform supports additional sensors/messages, then publishing can happen in this
  /// method.
  /// Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  virtual bool8_t update(std::chrono::nanoseconds timeout) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages
  /// Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool8_t send_state_command(const VehicleStateCommand & msg) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages.
  /// Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool8_t send_control_command(const VehicleControlCommand & msg) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages.
  /// Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool8_t send_control_command(const AckermannControlCommand & msg) = 0;
  /// Send the state command to the vehicle platform. May require translation into a
  /// vehicle-specific representation and sending multiple messages.
  /// Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  virtual bool8_t send_control_command(const RawControlCommand & msg) = 0;
  /// Respond to a request to change the autonomy mode. This should only fail if
  /// changing the mode on the actual low-level autonomy interface fails.
  /// Exceptions may be thrown on errors
  /// \param[in] request The requested mode
  /// \return false If changing the mode failed in some way, true otherwise
  virtual bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) = 0;

  /// Get the most recent state of the vehicle. The State should be assumed to be constant unless
  /// data from the vehicle platform implies a state should be changed. For example, if the gear
  /// state is drive, the StateReport should be in drive until the vehicle platform reports that
  /// it is in neutral or some other gear state.
  /// \return A StateReport message intended to be published.
  const VehicleStateReport & get_state_report() const noexcept;
  /// Get the most recent odomoetry of the vehicle
  /// \return A Odometry message intended to be published.
  const VehicleOdometry & get_odometry() const noexcept;
  /// \brief Get the most recent state of the headlights feature.
  /// \return A HeadlightsReport message intended to be published.
  const HeadlightsReport & get_headlights_report() const noexcept;
  /// \brief Get the most recent state of the horn feature.
  /// \return A HornReport message intended to be published.
  const HornReport & get_horn_report() const noexcept;
  /// \brief Get the most recent state of the wipers feature.
  /// \return A WipersReport message intended to be published.
  const WipersReport & get_wipers_report() const noexcept;

  /// \brief Send the headlight control command to the vehicle platform.
  /// If this is not implemented for a specific vehicle but is called,
  /// a runtime error will be thrown.
  /// \param[in] msg The control command to send to the vehicle.
  virtual void send_headlights_command(const HeadlightsCommand & msg);
  /// \brief Send the horn control command to the vehicle platform.
  /// If this is not implemented for a specific vehicle but is called,
  /// a runtime error will be thrown.
  /// \param[in] msg The control command to send to the vehicle.
  virtual void send_horn_command(const HornCommand & msg);

  /// \brief Send the wipers control command to the vehicle platform.
  /// If this is not implemented for a specific vehicle but is called,
  /// a runtime error will be thrown.
  /// \param[in] msg The control command to send to the vehicle.
  virtual void send_wipers_command(const WipersCommand & msg);

  /// \brief Send the hazard lights control command to the vehicle platform.
  /// If this is not implemented for a specific vehicle but is called,
  /// a runtime error will be thrown.
  /// \param[in] msg The control command to send to the vehicle.
  virtual void send_hazard_lights_command(const HazardLightsCommand & msg);

protected:
  /// Get the underlying state report for modification
  VehicleStateReport & state_report() noexcept;
  /// Get the underlying odometry for modification
  VehicleOdometry & odometry() noexcept;
  /// Get the underlying headlight state for modification
  HeadlightsReport & headlights_report() noexcept;
  /// Get the underlying horn state for modification
  HornReport & horn_report() noexcept;
  /// Get the underlying wiper state for modification
  WipersReport & wipers_report() noexcept;

private:
  HeadlightsReport m_headlights_report{};
  HornReport m_horn_report{};
  WipersReport m_wipers_report{};
  VehicleStateReport m_state_report{};
  VehicleOdometry m_odometry{};
};  // class PlatformInterface
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__PLATFORM_INTERFACE_HPP_
