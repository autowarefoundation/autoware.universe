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

#ifndef LANE_CHANGE_PLANNER_STATE_MACHINE_H
#define LANE_CHANGE_PLANNER_STATE_MACHINE_H

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_planning_msgs/Route.h>
#include <lane_change_planner/state/state_base_class.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <ros/ros.h>
#include <memory>

namespace lane_change_planner
{
// enum State{
//   WAITING_LANE_CHANGE,
//   EXECUTING_LANE_CHANGE,
//   ABORTING_LANE_CHANGE,
//   FORCING_LANE_CHANGE,
// };

class StateMachine
{
public:
  StateMachine(
    const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr);
  void init();
  void init(const autoware_planning_msgs::Route & route);
  void updateState();
  autoware_planning_msgs::PathWithLaneId getPath() const;
  Status getStatus() const;

private:
  std::unique_ptr<StateBase> state_obj_ptr_;
  std::shared_ptr<DataManager> data_manager_ptr_;
  std::shared_ptr<RouteHandler> route_handler_ptr_;
  ros::Publisher path_marker_publisher_;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_MACHINE_H
