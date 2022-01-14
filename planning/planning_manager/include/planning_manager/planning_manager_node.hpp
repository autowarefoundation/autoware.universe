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

#include "planning_manager/srv/behavior_path_planner.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/had_map_route.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/planning_data.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

namespace planning_manager
{
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

  // clients
  rclcpp::Client<planning_manager::srv::BehaviorPathPlanner>::SharedPtr
    client_behavior_path_planner_;

  autoware_auto_planning_msgs::msg::HADMapRoute route_;
  planning_manager::msg::PlanningData planning_data_;
  autoware_auto_planning_msgs::msg::PathWithLaneId path_with_lane_id_;
  autoware_auto_planning_msgs::msg::Path path_;
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;

  void onRoute(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg);
};
}  // namespace planning_manager

#endif  // PLANNING_MANAGER__PLANNING_MANAGER_CORE_CPP_
