// Copyright 2020 the Autoware Foundation
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
/// \brief Input validation state machine
#ifndef VEHICLE_INTERFACE__SAFETY_STATE_MACHINE_HPP_
#define VEHICLE_INTERFACE__SAFETY_STATE_MACHINE_HPP_

#include <common/types.hpp>
#include <autoware_auto_vehicle_msgs/msg/wipers_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/horn_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_report.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <experimental/optional>
#include <algorithm>
#include <chrono>
#include <vector>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{
using autoware_auto_vehicle_msgs::msg::HeadlightsCommand;
using autoware_auto_vehicle_msgs::msg::HornCommand;
using autoware_auto_vehicle_msgs::msg::WipersCommand;
using MaybeStateCommand = std::experimental::optional<autoware_auto_vehicle_msgs::msg::VehicleStateCommand>;
using BasicControlCommand = autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
using Odometry = autoware_auto_vehicle_msgs::msg::VehicleOdometry;
using StateReport = autoware_auto_vehicle_msgs::msg::VehicleStateReport;
/// Simple wrapper for control command and state command together
class VEHICLE_INTERFACE_PUBLIC Command
{
public:
  /// Constructor
  Command(
    const BasicControlCommand & control = BasicControlCommand{},
    const MaybeStateCommand & state = MaybeStateCommand{})
  : m_control{control}, m_state{state} {}

  /// Getter
  const BasicControlCommand & control() const noexcept {return m_control;}
  /// Getter
  const MaybeStateCommand & state() const noexcept {return m_state;}

private:
  BasicControlCommand m_control;
  MaybeStateCommand m_state;
};  // class Command

/// Denotes one of the behaviors that the state machine has done; either a notification that it has
/// done something (Info), or a warning that something bad has been detected
enum class StateMachineReport : uint8_t
{
  CLAMP_PAST_THRESHOLD,  ///< Clamped a command that was way out of bounds
  BAD_STATE,  ///< A state command was not one of the specified constants
  WIPERS_ON_HEADLIGHTS_ON,  ///< Turn on headlights because you turned on wipers
  REMOVE_GEAR_COMMAND,  ///< Converted gear command to GEAR_NO_COMMAND because you're going too fast
  HIGH_FREQUENCY_ACCELERATION_COMMAND,  ///< Acceleration has high frequency components
  HIGH_FREQUENCY_STEER_COMMAND,  ///< Steering has high frequency components
  HIGH_FREQUENCY_VELOCITY_REPORT,  ///< Reported velocity has high frequency components
  HIGH_FREQUENCY_STEER_REPORT,  ///< Reported steering has high frequency components
  STATE_TRANSITION_TIMEOUT  ///< Commanded state transition didn't happen
};  // enum class StateMachineWarning

// TODO(c.ho) concept comparable
/// A simple class representing 1D limits and a threshold value for when something is considered
/// warning worthy.
template<typename T>
class VEHICLE_INTERFACE_PUBLIC Limits
{
public:
  /// Constructor
  /// \throw std::domain_error if min >= max
  Limits(T min, T max, T threshold)
  : m_min{min}, m_max{max}, m_threshold{threshold}
  {
    if (min >= max) {
      throw std::domain_error{"min >= max"};
    }
  }

  T min() const noexcept {return m_min;}
  T max() const noexcept {return m_max;}
  T threshold() const noexcept {return m_threshold;}
  ///  Clamps value to max/min range; return true if value is threshold past limits
  bool8_t clamp_warn(T & value) const noexcept
  {
    const auto value_raw = value;
    value = std::min(std::max(m_min, value_raw), m_max);
    return std::abs(value - value_raw) >= m_threshold;
  }

private:
  T m_min;
  T m_max;
  T m_threshold;
};

/// Configuration class for SafetyStateMachine
class VEHICLE_INTERFACE_PUBLIC StateMachineConfig
{
public:
  using VelocityT = decltype(Odometry::velocity_mps);
  using AccelT = decltype(BasicControlCommand::long_accel_mps2);
  using AccelLimits = Limits<AccelT>;
  using FrontWheelLimits = Limits<decltype(BasicControlCommand::front_wheel_angle_rad)>;

  /// Constructor
  /// \param[in] gear_shift_velocity_threshold Gear shifts between directions (i.e. park->drive,
  /// low->reverse are allowed only if the magnitude of velocity is below this threshold. Must be
  /// positive
  /// \param[in] accel_limits Max/min value for command acceleration
  /// \param[in] front_wheel_limits Max/min value for commanded wheel angles
  /// \param[in] time_step Characteristic time step of the system: used for computing timeout
  /// acceleration commands and when to do autonomous gear shifting. Must be postiive
  /// \param[in] timeout_accel_mps2 Magnitude of acceleration to use to bring the vehicle to a stop
  /// when the vehicle interface has timed out. Must be positive
  /// \param[in] state_transition_timeout If a state component has not changed within this
  /// period, then a warning is raised. Must be positive and bigger than time_step.
  /// \param[in] auto_gear_shift_accel_deadzone_mps2 Acceleration must be bigger than this magnitude
  /// to induce an automatic gear shift. Must be positive.
  /// \throw std::domain_error If gear_shift_velocity_threshold is negative
  /// \throw std::domain_error If time_step is non-positive
  /// \throw std::domain_error If timeout_accel_mps2 is non-positive, or outside of accel_limits
  /// \throw std::domain_error If state_transition_timeout is smaller than time_step
  /// \throw std::domain_error If auto_gear_shift_accel_deadzone_mps2 is non-positive
  StateMachineConfig(
    const VelocityT gear_shift_velocity_threshold,
    const AccelLimits & accel_limits,
    const FrontWheelLimits & front_wheel_limits,
    const std::chrono::nanoseconds time_step,
    const AccelT timeout_accel_mps2,
    const std::chrono::nanoseconds state_transition_timeout,
    const AccelT auto_gear_shift_accel_deadzone_mps2
  );

  VelocityT gear_shift_velocity_threshold() const noexcept;
  const AccelLimits & accel_limits() const noexcept;
  const FrontWheelLimits & front_wheel_limits() const noexcept;
  std::chrono::nanoseconds time_step() const noexcept;
  AccelT timeout_acceleration() const noexcept;
  std::chrono::nanoseconds state_transition_timeout() const noexcept;
  AccelT auto_gear_shift_accel_deadzone() const noexcept;

private:
  VelocityT m_gear_shift_velocity_threshold;
  AccelLimits m_accel_limits;
  FrontWheelLimits m_front_wheel_limits;
  std::chrono::nanoseconds m_time_step;
  AccelT m_timeout_acceleration;
  std::chrono::nanoseconds m_state_transition_timeout;
  AccelT m_auto_gear_shift_accel_deadzone;
};  // class StateMachineConfig

/// Implements stateful behavior for VehicleInterfaceNode
/// Primary function is to ensure vehicle platform does not get
/// conflicting commands
class VEHICLE_INTERFACE_PUBLIC SafetyStateMachine
{
public:
  using Reports = std::vector<StateMachineReport>;  // TODO(c.ho) maybe a set instead
  /// Constructor
  explicit SafetyStateMachine(const StateMachineConfig & config);
  /// Ensure given commands are consistent and not dangerous. See design document for
  /// full list of behaviors.
  /// \param[in] command The raw input commands
  /// \return Commands which are consistent and safe (within the scope of the vehicle interface)
  Command compute_safe_commands(const Command & command);
  /// Update the state of the state machine
  /// \param[in] odom The odometry of the vehicle
  /// \param[in] state The state of the vehicle
  void update(const Odometry & odom, const StateReport & state);
  /// Compute safety fallback commands that should be executed on a timeout
  /// \return The safety fallback command for the current state
  Command timeout_commands() const noexcept;
  /// Get list of warnings state machine saw and adjusted
  const Reports & reports() const noexcept;
  /// Get config object
  const StateMachineConfig & get_config() const noexcept;

private:
  using VSC = autoware_auto_vehicle_msgs::msg::VehicleStateCommand;
  using MaybeEnum = std::experimental::optional<decltype(VSC::blinker)>;
  //lint -save -e9150 NOLINT Pure aggregate and only used internally; no constraints on state
  template<typename T>
  struct ValueStamp
  {
    T value;
    std::chrono::system_clock::time_point stamp{std::chrono::system_clock::time_point::min()};
  };
  struct StateChangeRequests
  {
    ValueStamp<decltype(VSC::gear)> gear;
    ValueStamp<decltype(VSC::blinker)> blinker;
    ValueStamp<decltype(WipersCommand::command)> wiper;
    ValueStamp<decltype(HeadlightsCommand::command)> headlight;
    ValueStamp<decltype(VSC::mode)> mode;
    ValueStamp<decltype(VSC::hand_brake)> hand_brake;
    ValueStamp<decltype(HornCommand::active)> horn;
  };  // struct StateChangeRequest
  //lint -restore NOLINT
  // Make sure uint members are within range--otherwise set to NO_COMMAND
  VEHICLE_INTERFACE_LOCAL VSC sanitize(const VSC & msg) const;
  // Turn on headlights if the wipers are being turned on
  VEHICLE_INTERFACE_LOCAL static MaybeEnum headlights_on_if_wipers_on(const VSC & in);
  // Apply gear shift if velocity is small and commanded acceleration is big enough
  VEHICLE_INTERFACE_LOCAL uint8_t automatic_gear_shift(
    const BasicControlCommand control,
    const VSC & state) const;
  // Remove "big" gear shifts if velocity is too large in magnitude
  VEHICLE_INTERFACE_LOCAL bool8_t bad_gear_shift(const VSC & in) const;
  // Clamp control values; warn if wildly out of range
  VEHICLE_INTERFACE_LOCAL BasicControlCommand clamp(BasicControlCommand in) const;
  // Add/overwrite any new state change requests
  VEHICLE_INTERFACE_LOCAL void cache_state_change_request(const VSC & in);
  // Clear any state changes that have happened, or warn on timeout
  VEHICLE_INTERFACE_LOCAL void check_state_change(const StateReport & in);

  StateMachineConfig m_config;
  Odometry m_odometry{};
  StateReport m_state{};
  StateChangeRequests m_requests{};
  mutable Reports m_reports{};
};  // class SafetyStateMachine
}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__SAFETY_STATE_MACHINE_HPP_
