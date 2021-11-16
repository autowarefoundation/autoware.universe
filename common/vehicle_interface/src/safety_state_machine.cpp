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
#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <time_utils/time_utils.hpp>

#include <cmath>
#include <limits>

#include "vehicle_interface/safety_state_machine.hpp"

#include "autoware_auto_msgs/msg/headlights_command.hpp"
#include "autoware_auto_msgs/msg/headlights_report.hpp"
#include "autoware_auto_msgs/msg/horn_command.hpp"
#include "autoware_auto_msgs/msg/horn_report.hpp"
#include "autoware_auto_msgs/msg/wipers_command.hpp"
#include "autoware_auto_msgs/msg/wipers_report.hpp"

using autoware::common::types::bool8_t;
namespace comp = autoware::common::helper_functions::comparisons;

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{
namespace
{
using VSC = autoware_auto_msgs::msg::VehicleStateCommand;
using VSR = StateReport;
using autoware_auto_msgs::msg::HeadlightsCommand;
using autoware_auto_msgs::msg::HeadlightsReport;
using autoware_auto_msgs::msg::HornCommand;
using autoware_auto_msgs::msg::HornReport;
using autoware_auto_msgs::msg::WipersCommand;
using autoware_auto_msgs::msg::WipersReport;
static_assert(VSC::BLINKER_OFF == VSR::BLINKER_OFF, "BLINKER_OFF!=");
static_assert(VSC::BLINKER_LEFT == VSR::BLINKER_LEFT, "BLINKER_LEFT !=");
static_assert(VSC::BLINKER_RIGHT == VSR::BLINKER_RIGHT, "BLINKER_RIGHT !=");
static_assert(VSC::BLINKER_HAZARD == VSR::BLINKER_HAZARD, "BLINKER_HAZARD !=");
static_assert(HeadlightsCommand::DISABLE == VSR::HEADLIGHT_OFF, "HEADLIGHT_OFF !=");
static_assert(HeadlightsCommand::ENABLE_LOW == VSR::HEADLIGHT_ON, "HEADLIGHT_ON !=");
static_assert(HeadlightsCommand::ENABLE_HIGH == VSR::HEADLIGHT_HIGH, "HEADLIGHT_HIGH !=");
static_assert(WipersCommand::DISABLE == WipersReport::DISABLE, "DISABLE !=");
static_assert(WipersCommand::ENABLE_LOW == WipersReport::ENABLE_LOW, "ENABLE_LOW !=");
static_assert(WipersCommand::ENABLE_HIGH == WipersReport::ENABLE_HIGH, "ENABLE_HIGH !=");
static_assert(WipersCommand::ENABLE_CLEAN == WipersReport::ENABLE_CLEAN, "ENABLE_CLEAN !=");
static_assert(VSC::GEAR_DRIVE == VSR::GEAR_DRIVE, "GEAR_DRIVE !=");
static_assert(VSC::GEAR_REVERSE == VSR::GEAR_REVERSE, "GEAR_REVERSE !=");
static_assert(VSC::GEAR_PARK == VSR::GEAR_PARK, "GEAR_PARK !=");
static_assert(VSC::GEAR_LOW == VSR::GEAR_LOW, "GEAR_LOW !=");
static_assert(VSC::GEAR_NEUTRAL == VSR::GEAR_NEUTRAL, "GEAR_NEUTRAL !=");
static_assert(VSC::MODE_AUTONOMOUS == VSR::MODE_AUTONOMOUS, "MODE_AUTONOMOUS !=");
static_assert(VSC::MODE_MANUAL == VSR::MODE_MANUAL, "MODE_MANUAL !=");
}  // namespace

using VCC = autoware_auto_msgs::msg::VehicleControlCommand;

////////////////////////////////////////////////////////////////////////////////
StateMachineConfig::StateMachineConfig(
  const VelocityT gear_shift_velocity_threshold,
  const AccelLimits & accel_limits,
  const FrontWheelLimits & front_wheel_limits,
  const std::chrono::nanoseconds time_step,
  const AccelT timeout_accel_mps2,
  const std::chrono::nanoseconds state_transition_timeout,
  const AccelT auto_gear_shift_accel_deadzone_mps2)
: m_gear_shift_velocity_threshold{gear_shift_velocity_threshold},
  m_accel_limits{accel_limits},
  m_front_wheel_limits{front_wheel_limits},
  m_time_step{time_step},
  m_timeout_acceleration{timeout_accel_mps2},
  m_state_transition_timeout{state_transition_timeout},
  m_auto_gear_shift_accel_deadzone{auto_gear_shift_accel_deadzone_mps2}
{
  if (VelocityT{} >= gear_shift_velocity_threshold) {
    throw std::domain_error{"Gear shift velocity threshold must be positive"};
  }
  if (decltype(time_step)::zero() >= time_step) {
    throw std::domain_error{"Time step must be positive"};
  }
  if (AccelT{} >= timeout_accel_mps2) {
    throw std::domain_error{"Timeout acceleration must be > 0"};
  }
  if ((timeout_accel_mps2 < accel_limits.min()) || (timeout_accel_mps2 > accel_limits.max())) {
    throw std::domain_error{"Timeout acceleration outside of acceleration limits"};
  }
  if (state_transition_timeout < time_step) {
    throw std::domain_error{"State transition timeout must be > time_step"};
  }
  if (AccelT{} >= auto_gear_shift_accel_deadzone_mps2) {
    throw std::domain_error{"Gear shift acceleration deadzone must be > 0"};
  }
}

StateMachineConfig::VelocityT StateMachineConfig::gear_shift_velocity_threshold() const noexcept
{
  return m_gear_shift_velocity_threshold;
}

const StateMachineConfig::AccelLimits & StateMachineConfig::accel_limits() const noexcept
{
  return m_accel_limits;
}

const StateMachineConfig::FrontWheelLimits & StateMachineConfig::front_wheel_limits() const noexcept
{
  return m_front_wheel_limits;
}

std::chrono::nanoseconds StateMachineConfig::time_step() const noexcept {return m_time_step;}

StateMachineConfig::AccelT StateMachineConfig::timeout_acceleration() const noexcept
{
  return m_timeout_acceleration;
}

std::chrono::nanoseconds StateMachineConfig::state_transition_timeout() const noexcept
{
  return m_state_transition_timeout;
}

StateMachineConfig::AccelT StateMachineConfig::auto_gear_shift_accel_deadzone() const noexcept
{
  return m_auto_gear_shift_accel_deadzone;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
SafetyStateMachine::SafetyStateMachine(const StateMachineConfig & config)
: m_config{config}
{
  m_reports.reserve(10U);  // Number of enums
  {  // initialize state to something sensible
    m_state.gear = VSR::GEAR_DRIVE;
    m_state.blinker = VSR::BLINKER_OFF;
    m_state.wiper = VSR::WIPER_OFF;
    m_state.headlight = VSR::HEADLIGHT_OFF;
    m_state.mode = VSR::MODE_NOT_READY;
    m_state.horn = false;
    m_state.hand_brake = false;
  }
}

////////////////////////////////////////////////////////////////////////////////
Command SafetyStateMachine::compute_safe_commands(const Command & command)
{
  m_reports.clear();
  const auto control = clamp(command.control());
  MaybeStateCommand state{command.state()};
  if (command.state()) {
    // Sanitize state command
    state = sanitize(command.state().value());
    // Apply headlight logic
    const auto new_headlight = headlights_on_if_wipers_on(state.value());
    if (new_headlight) {
      state->headlight = new_headlight.value();
      m_reports.emplace_back(StateMachineReport::WIPERS_ON_HEADLIGHTS_ON);
    }
    // Apply automatic gear shift logic: no command if already in that state
    {
      const auto requested_gear = automatic_gear_shift(control, state.value());
      state->gear = (requested_gear == m_state.gear) ? VSC::GEAR_NO_COMMAND : requested_gear;
    }
    // Apply gear check logic
    if (bad_gear_shift(state.value())) {
      state->gear = VSC::GEAR_NO_COMMAND;
      m_reports.emplace_back(StateMachineReport::REMOVE_GEAR_COMMAND);
    }
    // Add requests to queue
    cache_state_change_request(state.value());
  } else {
    // Check if you need to do gear shifting
    //lint -e{1793} NOLINT nonconst member on temp ok: named parameter idiom
    const auto dummy_state = VSC{}.set__gear(m_state.gear);
    const auto requested_gear = automatic_gear_shift(control, dummy_state);
    if (requested_gear != m_state.gear) {
      state = VSC{};
      state->stamp = control.stamp;
      state->gear = requested_gear;
    }
  }
  return Command{control, state};
}

////////////////////////////////////////////////////////////////////////////////
void SafetyStateMachine::update(const Odometry & odom, const StateReport & state)
{
  m_reports.clear();
  // TODO(c.ho) sanitize StateReport
  m_odometry = odom;
  m_state = state;
  // Check if state update has been satisfied
  check_state_change(state);
}

////////////////////////////////////////////////////////////////////////////////
Command SafetyStateMachine::timeout_commands() const noexcept
{
  //lint -e{1793} NOLINT temporary ok: named parameter idiom
  const auto state = VSC{}.set__blinker(VSC::BLINKER_HAZARD);
  using Accel = decltype(BasicControlCommand::long_accel_mps2);
  auto acceleration = -m_odometry.velocity_mps /
    std::chrono::duration_cast<std::chrono::duration<Accel>>(m_config.time_step()).count();
  (void)m_config.accel_limits().clamp_warn(acceleration);
  //lint -e{1793} NOLINT temporary ok: named parameter idiom
  const auto control = BasicControlCommand{}.set__long_accel_mps2(acceleration);
  return Command{control, state};
}

////////////////////////////////////////////////////////////////////////////////
const SafetyStateMachine::Reports & SafetyStateMachine::reports() const noexcept
{
  return m_reports;
}

////////////////////////////////////////////////////////////////////////////////
const StateMachineConfig & SafetyStateMachine::get_config() const noexcept
{
  return m_config;
}

////////////////////////////////////////////////////////////////////////////////
SafetyStateMachine::MaybeEnum SafetyStateMachine::headlights_on_if_wipers_on(const VSC & in)
{
  MaybeEnum ret{};
  switch (in.wiper) {
    case WipersCommand::ENABLE_LOW:
    case WipersCommand::ENABLE_HIGH:
      switch (in.headlight) {
        case HeadlightsCommand::DISABLE:
        case HeadlightsCommand::NO_COMMAND:
          ret = MaybeEnum{HeadlightsCommand::ENABLE_LOW};
          break;
        case HeadlightsCommand::ENABLE_LOW:
        case HeadlightsCommand::ENABLE_HIGH:
        default:  // throw on other cases?
          break;
      }
      break;
    case WipersCommand::DISABLE:
    case WipersCommand::ENABLE_CLEAN:
    case WipersCommand::NO_COMMAND:
    default:  // Throw on other cases?
      break;
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool8_t SafetyStateMachine::bad_gear_shift(const VSC & in) const
{
  // Doing nothing -> no check needed
  if (VSC::GEAR_NO_COMMAND == in.gear) {
    return false;
  }
  // Low velocity -> no check needed
  const auto velocity_over_threshold =
    (m_odometry.velocity_mps > m_config.gear_shift_velocity_threshold()) ||
    (m_odometry.velocity_mps < -m_config.gear_shift_velocity_threshold());
  if (!velocity_over_threshold) {
    return false;
  }
  // If start and end start are not both forward -> bad
  const auto going_forward =
    (StateReport::GEAR_DRIVE == m_state.gear) || (StateReport::GEAR_LOW == m_state.gear);
  const auto target_forward = (VSC::GEAR_DRIVE == in.gear) || (VSC::GEAR_LOW == in.gear);
  if (going_forward != target_forward) {
    return true;
  }
  // Otherwise if start and end state (park or reverse) are not the same -> bad
  static_assert(VSC::GEAR_REVERSE == StateReport::GEAR_REVERSE, "VSC::reverse != VSR::reverse");
  static_assert(VSC::GEAR_PARK == StateReport::GEAR_PARK, "VSC::park != VSR::park");
  return m_state.gear != in.gear;
}

////////////////////////////////////////////////////////////////////////////////
uint8_t SafetyStateMachine::automatic_gear_shift(
  const BasicControlCommand control,
  const VSC & state) const
{
  // If you're specifically asking for a park, I'll respect that and let other logic check
  if (VSC::GEAR_PARK == state.gear) {
    return state.gear;
  }
  // Small acceleration -> no check; let other logic handle
  using Real = decltype(control.long_accel_mps2);
  if (std::fabs(control.long_accel_mps2) < m_config.auto_gear_shift_accel_deadzone()) {
    return state.gear;
  }
  // High velocity -> no check needed: let other logic handle
  const auto velocity = m_odometry.velocity_mps;
  const auto velocity_over_threshold = (velocity > m_config.gear_shift_velocity_threshold()) ||
    (velocity < -m_config.gear_shift_velocity_threshold());
  if (velocity_over_threshold) {
    return state.gear;
  }
  // If velocity and acceleration are in the same direction -> no check; continue if 0 though
  if (Real{} < (control.long_accel_mps2 * velocity)) {
    return state.gear;
  }
  // Check if commanded acceleration will switch your sign
  const auto dv =
    control.long_accel_mps2 *
    std::chrono::duration_cast<std::chrono::duration<Real>>(m_config.time_step()).count();
  const auto v_next = dv + velocity;
  const auto accel_switches_sign = Real{} > (velocity * v_next);
  // Nonzero velocity and acceleration doesn't switch sign -> stay as is
  constexpr auto EPS = std::numeric_limits<Real>::epsilon();
  if (!comp::abs_eq_zero(velocity, EPS) && (!accel_switches_sign)) {
    return state.gear;
  }
  // Zero velocity -> any acceleration should be a gear shift
  // Return drive or reverse according to acceleration sign
  return (control.long_accel_mps2 > Real{}) ? VSC::GEAR_DRIVE : VSC::GEAR_REVERSE;
}

////////////////////////////////////////////////////////////////////////////////
BasicControlCommand SafetyStateMachine::clamp(BasicControlCommand in) const
{
  auto warn = false;
  warn = m_config.accel_limits().clamp_warn(in.long_accel_mps2) || warn;
  warn = m_config.front_wheel_limits().clamp_warn(in.front_wheel_angle_rad) || warn;
  if (warn) {
    m_reports.emplace_back(StateMachineReport::CLAMP_PAST_THRESHOLD);
  }
  return in;
}

////////////////////////////////////////////////////////////////////////////////
void SafetyStateMachine::cache_state_change_request(const VSC & in)
{
  const auto stamp = ::time_utils::from_message(in.stamp);
  // States
  const auto update_request = [stamp](auto no_command, auto & command, auto state, auto & request) {
      if ((no_command != command) && (command != state)) {
        request = {command, stamp};
      }
    };
  update_request(WipersCommand::NO_COMMAND, in.wiper, m_state.wiper, m_requests.wiper);
  update_request(VSC::BLINKER_NO_COMMAND, in.blinker, m_state.blinker, m_requests.blinker);
  update_request(VSC::GEAR_NO_COMMAND, in.gear, m_state.gear, m_requests.gear);
  update_request(
    HeadlightsCommand::NO_COMMAND, in.headlight, m_state.headlight,
    m_requests.headlight);
  update_request(VSC::MODE_NO_COMMAND, in.mode, m_state.mode, m_requests.mode);
  // Flags
  const auto update_flag_request = [stamp](auto command, auto state, auto & request) -> void {
      if (command != state) {
        request = {command, stamp};
      }
    };
  update_flag_request(in.horn, m_state.horn, m_requests.horn);
  update_flag_request(in.hand_brake, m_state.hand_brake, m_requests.hand_brake);
}

////////////////////////////////////////////////////////////////////////////////
void SafetyStateMachine::check_state_change(const StateReport & in)
{
  auto timeout = false;
  const auto stamp = ::time_utils::from_message(in.stamp) - m_config.state_transition_timeout();
  // Check states
  const auto check_state = [&timeout, stamp](auto & request, auto state, auto no_command) -> void {
      if (no_command == request.value) {
        return;
      }
      if (request.value == state) {
        request = {no_command, decltype(stamp)::min()};
      }
      if (stamp > request.stamp) {
        timeout = true;
      }
    };
  check_state(m_requests.gear, in.gear, VSC::GEAR_NO_COMMAND);
  check_state(m_requests.blinker, in.blinker, VSC::BLINKER_NO_COMMAND);
  check_state(m_requests.wiper, in.wiper, WipersCommand::NO_COMMAND);
  check_state(m_requests.headlight, in.headlight, HeadlightsCommand::NO_COMMAND);
  check_state(m_requests.mode, in.mode, VSC::MODE_NO_COMMAND);
  // Check flags
  const auto check_flag = [&timeout, stamp](auto request, auto state) -> void {
      if (request.value == state) {
        return;
      }
      if (stamp > request.stamp) {
        timeout = true;
      }
    };
  //lint -save -e523 NOLINT false positive: lambda affects timeout which is reference captured
  check_flag(m_requests.horn, in.horn);
  check_flag(m_requests.hand_brake, in.hand_brake);
  //lint -restore NOLINT
  // Warn on timeout
  if (timeout) {
    m_reports.emplace_back(StateMachineReport::STATE_TRANSITION_TIMEOUT);
  }
}

////////////////////////////////////////////////////////////////////////////////
SafetyStateMachine::VSC SafetyStateMachine::sanitize(const VSC & msg) const
{
  auto did_sanitize = false;
  VSC ret{msg};
  // Headlights
  switch (msg.headlight) {
    case HeadlightsCommand::NO_COMMAND:
    case HeadlightsCommand::DISABLE:
    case HeadlightsCommand::ENABLE_LOW:
    case HeadlightsCommand::ENABLE_HIGH:
      break;
    default:
      did_sanitize = true;
      ret.headlight = HeadlightsCommand::NO_COMMAND;
      break;
  }
  // Blinker
  switch (msg.blinker) {
    case VSC::BLINKER_NO_COMMAND:
    case VSC::BLINKER_OFF:
    case VSC::BLINKER_LEFT:
    case VSC::BLINKER_RIGHT:
    case VSC::BLINKER_HAZARD:
      break;
    default:
      did_sanitize = true;
      ret.blinker = VSC::BLINKER_NO_COMMAND;
      break;
  }
  // Wipers
  switch (msg.wiper) {
    case WipersCommand::ENABLE_LOW:
    case WipersCommand::ENABLE_HIGH:
    case WipersCommand::DISABLE:
    case WipersCommand::ENABLE_CLEAN:
    case WipersCommand::NO_COMMAND:
      break;
    default:
      did_sanitize = true;
      ret.wiper = WipersCommand::NO_COMMAND;
      break;
  }
  // Gear
  switch (msg.gear) {
    case VSC::GEAR_NO_COMMAND:
    case VSC::GEAR_DRIVE:
    case VSC::GEAR_PARK:
    case VSC::GEAR_REVERSE:
    case VSC::GEAR_NEUTRAL:
    case VSC::GEAR_LOW:
      break;
    default:
      did_sanitize = true;
      ret.gear = VSC::GEAR_NO_COMMAND;
      break;
  }
  // Mode
  switch (msg.mode) {
    case VSC::MODE_NO_COMMAND:
    case VSC::MODE_AUTONOMOUS:
    case VSC::MODE_MANUAL:
      break;
    default:
      did_sanitize = true;
      ret.mode = VSC::MODE_NO_COMMAND;
      break;
  }

  if (did_sanitize) {
    m_reports.emplace_back(StateMachineReport::BAD_STATE);
  }

  return ret;
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware
