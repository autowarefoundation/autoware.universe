// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_
#define SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <scene_module/out_of_lane/out_of_lane_utils.hpp>
#include <scene_module/scene_module_interface.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
class OutOfLaneModule : public SceneModuleInterface
{
  using PlannerParam = out_of_lane_utils::PlannerParam;
  using DebugData = out_of_lane_utils::DebugData;

public:
  OutOfLaneModule(
    const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
    PlannerParam planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief insert stop or slow down points to prevent dangerously entering another lane
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  // Parameter
  PlannerParam params_;
  // TODO(Maxime): using a raw ptr to shared pointer is BAD but I did not find how to update
  // planner_data otherwise
  const std::shared_ptr<const PlannerData> * planner_data_;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stop_watch_;
  std::vector<lanelet::BasicPolygon2d> partition_lanelets_;

protected:
  int64_t module_id_{};

  // Debug
  mutable DebugData debug_data_;

  std::shared_ptr<motion_utils::VirtualWallMarkerCreator> virtual_wall_marker_creator_ =
    std::make_shared<motion_utils::VirtualWallMarkerCreator>();
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_
