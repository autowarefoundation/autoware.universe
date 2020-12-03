// Copyright 2020 Tier IV, Inc.
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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/dynamic_object_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <scene_module/crosswalk/scene_crosswalk.hpp>
#include <scene_module/crosswalk/util.hpp>
#include <scene_module/scene_module_interface.hpp>

class WalkwayModule : public SceneModuleInterface
{
public:
public:
  struct PlannerParam
  {
    double stop_margin;
  };
  WalkwayModule(
    const int64_t module_id, const lanelet::ConstLanelet & walkway,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  enum class State { APPROACH, STOP, SURPASSED };

  lanelet::ConstLanelet walkway_;
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
