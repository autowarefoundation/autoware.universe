/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <lane_change_planner/state/aborting_lane_change.h>

namespace lane_change_planner
{
AbortingLaneChangeState::AbortingLaneChangeState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}
State AbortingLaneChangeState::getCurrentState() const { return State::ABORTING_LANE_CHANGE; }

void AbortingLaneChangeState::entry() {}

autoware_planning_msgs::PathWithLaneId AbortingLaneChangeState::getPath() const
{
  return status_.lane_follow_path;
}

void AbortingLaneChangeState::update() {}

State AbortingLaneChangeState::getNextState() const
{
  if (hasReturnedToOriginalLane()) {
    return State::FOLLOWING_LANE;
  }
  return State::ABORTING_LANE_CHANGE;
}

bool AbortingLaneChangeState::hasReturnedToOriginalLane() const { return true; }
}  // namespace lane_change_planner
