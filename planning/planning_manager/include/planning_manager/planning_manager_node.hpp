// Copyright 2022 Tier IV, Inc.
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

#ifndef PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_
#define PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_

#include "planning_manager/srv/behavior_path_planner_plan.hpp"
#include "planning_manager/srv/behavior_path_planner_validate.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/had_map_route.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/planning_data.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

namespace planning_manager
{
using HADMapRoute = autoware_auto_planning_msgs::msg::HADMapRoute;
using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;
using Path = autoware_auto_planning_msgs::msg::Path;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using BehaviorPathPlannerPlan = planning_manager::srv::BehaviorPathPlannerPlan;
using BehaviorPathPlannerValidate = planning_manager::srv::BehaviorPathPlannerValidate;

class PlanningManagerNode : public rclcpp::Node
{
public:
  explicit PlanningManagerNode(const rclcpp::NodeOptions & node_options);
  void run();

private:
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_sub_;

  // NOTE: callback group for client must be member variable
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // clients
  rclcpp::Client<planning_manager::srv::BehaviorPathPlannerPlan>::SharedPtr
    client_behavior_path_planner_plan_;
  rclcpp::Client<planning_manager::srv::BehaviorPathPlannerValidate>::SharedPtr
    client_behavior_path_planner_validate_;

  bool is_showing_debug_info_;

  autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr route_;
  planning_manager::msg::PlanningData planning_data_;
  autoware_auto_planning_msgs::msg::PathWithLaneId path_with_lane_id_;
  autoware_auto_planning_msgs::msg::Path path_;
  autoware_auto_planning_msgs::msg::Trajectory behavior_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory motion_trajectory_;

  void onRoute(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg);

  Trajectory planTrajectory(HADMapRoute route);
  Trajectory optimizeVelocity(Trajectory traj);
  void validateTrajectory(Trajectory traj);
  void publishTraajectory();
  void publishDiagnostics();
};
}  // namespace planning_manager

#endif  // PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_
