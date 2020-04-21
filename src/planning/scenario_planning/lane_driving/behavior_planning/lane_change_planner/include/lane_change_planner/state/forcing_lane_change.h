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

#ifndef LANE_CHANGE_PLANNER_STATE_FORCING_LANE_CHANGE_H
#define LANE_CHANGE_PLANNER_STATE_FORCING_LANE_CHANGE_H

#include <lane_change_planner/state/state_base_class.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <lanelet2_core/primitives/Lanelet.h>

namespace lane_change_planner
{
class ForcingLaneChangeState : public StateBase
{
private:
  // State transition conditions
  bool hasFinishedLaneChange() const;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped::ConstPtr current_twist_;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects_;

  lanelet::ConstLanelets original_lanes_;
  lanelet::ConstLanelets target_lanes_;

public:
  ForcingLaneChangeState(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
    const std::shared_ptr<RouteHandler> & route_handler_ptr);
  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
  autoware_planning_msgs::PathWithLaneId getPath() const override;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_FORCING_LANE_CHANGE_H
