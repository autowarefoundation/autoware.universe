/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <lane_change_planner/state/state_base_class.h>

namespace lane_change_planner
{
std::ostream & operator<<(std::ostream & ostream, const State & state)
{
  switch (state) {
    case State::NO_STATE:
      ostream << std::string("NO_STATE");
      break;
    case State::FOLLOWING_LANE:
      ostream << std::string("FOLLOWING_LANE");
      break;
    case State::EXECUTING_LANE_CHANGE:
      ostream << std::string("EXECUTING_LANE_CHANGE");
      break;
    case State::ABORTING_LANE_CHANGE:
      ostream << std::string("ABORTING_LANE_CHANGE");
      break;
    case State::FORCING_LANE_CHANGE:
      ostream << std::string("FORCING_LANE_CHANGE");
      break;
    case State::BLOCKED_BY_OBSTACLE:
      ostream << std::string("BLOCKED_BY_OBSTACLE");
      break;
    default:
      ostream << std::string("NO_STATE");
      break;
  }
  return ostream;
}

StateBase::StateBase(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: data_manager_ptr_(data_manager_ptr), route_handler_ptr_(route_handler_ptr), status_(status)
{
}

Status StateBase::getStatus() const { return status_; }
}  // namespace lane_change_planner
