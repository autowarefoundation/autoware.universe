// Copyright 2021 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_IN_PUBLIC_ROAD_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_IN_PUBLIC_ROAD_HPP_

#include <memory>
#include <string>
#include <vector>

#include "boost/optional.hpp"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"

#include "autoware_perception_msgs/msg/dynamic_object.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "scene_module/scene_module_interface.hpp"
#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"
#include "utilization/boost_geometry_helper.hpp"

namespace behavior_velocity_planner
{
class OcclusionSpotInPublicModule : public SceneModuleInterface
{
  using PlannerParam = occlusion_spot_utils::PlannerParam;

public:
  struct DebugData
  {
    std::string road_type = "public";
    double z;
    std::vector<lanelet::BasicPolygon2d> sidewalks;
    std::vector<occlusion_spot_utils::PossibleCollisionInfo> possible_collisions;
  };

  OcclusionSpotInPublicModule(
    const int64_t module_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief plan occlusion spot velocity
   */
  bool modifyPathVelocity(
    autoware_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr dynamic_objects_array_;

  // Parameter
  PlannerParam param_;

protected:
  int64_t module_id_;

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__SCENE_OCCLUSION_SPOT_IN_PUBLIC_ROAD_HPP_
