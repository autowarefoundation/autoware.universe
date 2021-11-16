// Copyright 2020 The Autoware Foundation
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

#include <memory>

#include "gtest/gtest.h"
#include "vehicle_interface/dbw_state_machine.hpp"

using autoware::drivers::vehicle_interface::DbwState;
using autoware::drivers::vehicle_interface::DbwStateMachine;

TEST(TestDbwStateMachine, StateMachineDisableFromEnableRequested) {
  const auto dbw_state = std::make_unique<DbwStateMachine>(3);

  // Should start in DISABLED
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);

  // Should remain false even if DBW says it's enabled
  // This avoids sending enabled messages when we don't intend to
  dbw_state->dbw_feedback(true);
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);
  ASSERT_FALSE(dbw_state->enabled());

  // Affirmitive request should transition to ENABLE_REQUESTED but enabled() should still be false
  dbw_state->user_request(true);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);
  ASSERT_FALSE(dbw_state->enabled());

  // When in ENABLE_REQUESTED, should ignore DBW feedback
  dbw_state->dbw_feedback(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);

  // When in ENABLE_REQUESTED, should ignore affirmitive user requests
  dbw_state->user_request(true);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);

  // Should return to disabled from any state if user requests it
  dbw_state->user_request(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);
}

TEST(TestDbwStateMachine, StateMachineDisableFromEnableSent) {
  const auto dbw_state = std::make_unique<DbwStateMachine>(3);

  // Request has been made, should be in ENABLE_REQUESTED
  dbw_state->user_request(true);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);

  // When in ENABLE_REQUESTED, should ignore further affirmitive requests
  dbw_state->user_request(true);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);

  // Once disabled commands have been sent, enabled() should return true
  // but should still be in ENABLE_REQUESTED state until an enabled = true
  // message is sent for each type
  ASSERT_FALSE(dbw_state->enabled());
  dbw_state->control_cmd_sent();
  ASSERT_FALSE(dbw_state->enabled());
  dbw_state->state_cmd_sent();
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_REQUESTED);

  // Should transition to ENABLE_SENT when enabled = true message
  // is sent for either message type
  dbw_state->control_cmd_sent();
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);
  dbw_state->state_cmd_sent();
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);

  // Should disable from ENABLE_SENT if DBW feedback says disabled
  // more times than feedback debounce count
  dbw_state->dbw_feedback(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);
  dbw_state->dbw_feedback(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);
  dbw_state->dbw_feedback(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);
  dbw_state->dbw_feedback(false);
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);
}

TEST(TestDbwStateMachine, StateMachineDisableFromEnabledUser) {
  const auto dbw_state = std::make_unique<DbwStateMachine>(3);

  dbw_state->user_request(true);
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);

  // Should transition to ENABLED if we get confirmation from DBW
  dbw_state->dbw_feedback(true);
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLED);

  // Should immediately transition back to DISABLED if user requests it
  dbw_state->user_request(false);
  ASSERT_FALSE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);
}

TEST(TestDbwStateMachine, StateMachineDisableFromEnabledDbw) {
  const auto dbw_state = std::make_unique<DbwStateMachine>(3);

  dbw_state->user_request(true);
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);

  // Should transition to ENABLED if we get confirmation from DBW
  dbw_state->dbw_feedback(true);
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLED);

  // Should immediately transition back to DISABLED if DBW reports it
  dbw_state->dbw_feedback(false);
  ASSERT_FALSE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::DISABLED);
}

TEST(TestDbwStateMachine, StateMachineStayEnabled) {
  const auto dbw_state = std::make_unique<DbwStateMachine>(3);

  dbw_state->user_request(true);
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLE_SENT);

  // Should transition to ENABLED if we get confirmation from DBW
  dbw_state->dbw_feedback(true);
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLED);

  // Should stay ENABLED unless user requests or DBW reports
  dbw_state->user_request(true);
  dbw_state->dbw_feedback(true);
  dbw_state->control_cmd_sent();
  dbw_state->state_cmd_sent();
  ASSERT_TRUE(dbw_state->enabled());
  ASSERT_EQ(dbw_state->get_state(), DbwState::ENABLED);
}
