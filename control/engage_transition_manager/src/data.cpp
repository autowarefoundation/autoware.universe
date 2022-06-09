// Copyright 2022 Autoware Foundation
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

#include "engage_transition_manager/state.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"


#include <algorithm>
#include <cmath>

namespace engage_transition_manager
{

uint8_t toMsg(const State s)
{
  if (s == State::NONE) {
    return OperationMode::NONE;
  } else if (s == State::REMOTE) {
    return OperationMode::REMOTE;
  } else if (s == State::DIRECT) {
    return OperationMode::DIRECT;
  } else if (s == State::LOCAL) {
    return OperationMode::LOCAL;
  } else if (s == State::TRANSITION_TO_AUTO) {
    return OperationMode::TRANSITION_TO_AUTO;
  } else if (s == State::AUTONOMOUS) {
    return OperationMode::AUTONOMOUS;
  }
  return OperationMode::NONE;
}

State toEnum(const OperationMode m)
{
  if (m.mode == OperationMode::NONE) {
    return State::NONE;
  } else if (m.mode == OperationMode::REMOTE) {
    return State::REMOTE;
  } else if (m.mode == OperationMode::DIRECT) {
    return State::DIRECT;
  } else if (m.mode == OperationMode::LOCAL) {
    return State::LOCAL;
  } else if (m.mode == OperationMode::TRANSITION_TO_AUTO) {
    return State::TRANSITION_TO_AUTO;
  } else if (m.mode == OperationMode::AUTONOMOUS) {
    return State::AUTONOMOUS;
  }
  return State::NONE;
}

bool isManual(const State s)
{
  if (s == State::NONE || s == State::REMOTE || s == State::DIRECT || s == State::LOCAL) {
    return true;
  } else {
    return false;
  }
}

bool isAuto(const State s)
{
  if (s == State::AUTONOMOUS) {
    return true;
  } else {
    return false;
  }
}

std::string toStr(const State s)
{
  if (s == State::NONE) {
    return "NONE";
  } else if (s == State::REMOTE) {
    return "REMOTE";
  } else if (s == State::DIRECT) {
    return "DIRECT";
  } else if (s == State::LOCAL) {
    return "LOCAL";
  } else if (s == State::TRANSITION_TO_AUTO) {
    return "TRANSITION_TO_AUTO";
  } else if (s == State::AUTONOMOUS) {
    return "AUTONOMOUS";
  } else {
    return "INVALID";
  }
}

}  // namespace engage_transition_manager
