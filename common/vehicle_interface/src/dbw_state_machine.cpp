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

#include "vehicle_interface/dbw_state_machine.hpp"

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

DbwStateMachine::DbwStateMachine(uint16_t dbw_disabled_debounce)
: m_first_control_cmd_sent{false},
  m_first_state_cmd_sent{false},
  m_disabled_feedback_count{0},
  DISABLED_FEEDBACK_THRESH{dbw_disabled_debounce},
  m_state{DbwState::DISABLED}
{
}

bool8_t DbwStateMachine::enabled() const
{
  return m_state == DbwState::ENABLED ||
         m_state == DbwState::ENABLE_SENT ||
         (m_state == DbwState::ENABLE_REQUESTED &&
         m_first_control_cmd_sent &&
         m_first_state_cmd_sent);
}

DbwState DbwStateMachine::get_state() const {return m_state;}

void DbwStateMachine::dbw_feedback(bool8_t enabled)
{
  if (enabled) {                             // DBW system says enabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_state = DbwState::ENABLED;
      m_disabled_feedback_count = 0;
    }
  } else {                                   // DBW system says disabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_disabled_feedback_count++;           // Increase debounce count

      if (m_disabled_feedback_count > DISABLED_FEEDBACK_THRESH) {  // check debounce
        disable_and_reset();
      }
    } else if (m_state == DbwState::ENABLED) {  // and state is ENABLED
      disable_and_reset();
    }
  }
}

void DbwStateMachine::control_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a control command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_control_cmd_sent = true;
  }
}

void DbwStateMachine::state_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a state command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_state_cmd_sent = true;
  }
}

void DbwStateMachine::user_request(bool8_t enable)
{
  if (enable) {                           // Enable is being requested
    if (m_state == DbwState::DISABLED) {  // Only change states if currently in DISABLED
      m_state = DbwState::ENABLE_REQUESTED;
    }
  } else {                               // Disable is being requested
    disable_and_reset();                 // Disable in any state if user requests it
  }
}

void DbwStateMachine::disable_and_reset()
{
  m_state = DbwState::DISABLED;
  m_first_control_cmd_sent = false;
  m_first_state_cmd_sent = false;
  m_disabled_feedback_count = 0;
}

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware
