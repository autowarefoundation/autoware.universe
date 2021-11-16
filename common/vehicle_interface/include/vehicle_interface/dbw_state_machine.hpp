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
/// \brief Base class for vehicle drivers
#ifndef VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_
#define VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_

#include <common/types.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <experimental/optional>
#include <cstdint>
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <utility>

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

enum class DbwState
{
  DISABLED = 0,
  ENABLE_REQUESTED = 1,
  ENABLE_SENT = 2,
  ENABLED = 3
};  // enum class DbwState

/// \brief Class for maintaining the DBW state
class VEHICLE_INTERFACE_PUBLIC DbwStateMachine
{
public:
  /// \brief Default constructor
  /// \param[in] dbw_disabled_debounce If state = ENABLE_SENT and DBW reports DISABLED, debounce this many msgs  // NOLINT
  explicit DbwStateMachine(uint16_t dbw_disabled_debounce);
  /// Destructor
  ~DbwStateMachine() = default;

  /// \brief Returns true if state is ENABLED, ENABLE_SENT, or ENABLE_REQUESTED with conditions
  bool8_t enabled() const;

  /// \brief Returns current internal state
  /// \return A DbwState object representing the current state
  DbwState get_state() const;

  /// \brief Notifies the state machine that feedback was received from the DBW system
  /// \param[in] enabled If true, DBW system reports enabled. If false, DBW system reports disabled
  void dbw_feedback(bool8_t enabled);

  /// \brief Notifies the state machine that a control command was sent to the DBW system
  void control_cmd_sent();

  /// \brief Notifies the state machine that a state command was sent to the DBW system
  void state_cmd_sent();

  /// \brief The user has requested the DBW system to enable (true) or disable (false)
  /// \param[in] enable If true, request enable. If false, request disable
  void user_request(bool8_t enable);

private:
  bool8_t m_first_control_cmd_sent;
  bool8_t m_first_state_cmd_sent;
  uint16_t m_disabled_feedback_count;
  const uint16_t DISABLED_FEEDBACK_THRESH;
  DbwState m_state;

  void disable_and_reset();
};  // class DbwStateMachine

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_
