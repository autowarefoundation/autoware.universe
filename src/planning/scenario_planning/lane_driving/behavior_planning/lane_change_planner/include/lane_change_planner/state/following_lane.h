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

#ifndef LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H
#define LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lane_change_planner/state/state_base_class.h>
#include <lanelet2_core/primitives/Primitive.h>

#include <memory>

namespace lane_change_planner
{
class FollowingLaneState : public StateBase
{
private:
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped::ConstPtr current_twist_;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects_;
  bool lane_change_approved_;
  bool force_lane_change_;
  lanelet::ConstLanelets current_lanes_;
  lanelet::ConstLanelets lane_change_lanes_;

  // State transition conditions
  bool isVehicleInPreferredLane() const;
  bool isTooCloseToDeadEnd() const;
  bool laneChangeForcedByOperator() const;
  bool isLaneBlocked(const lanelet::ConstLanelets & lanes) const;
  bool isLaneChangeReady() const;
  // minor conditions
  bool hasEnoughDistance() const;
  bool isLaneChangePathSafe() const;
  bool isLaneChangeApproved() const;
  bool isLaneChangeAvailable() const;

public:
  FollowingLaneState(
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

#endif  // LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H
