// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <memory>
#include <vector>
#include "lanelet2_core/primitives/BasicRegulatoryElements.h"

#include "lanelet2_extension/utility/utilities.hpp"

#include "scene_module/occlusion_spot/occlusion_spot_utils.hpp"
#include "scene_module/occlusion_spot/risk_predictive_braking.hpp"
#include "scene_module/occlusion_spot/scene_occlusion_spot_in_private_road.hpp"
#include "utilization/util.hpp"

namespace behavior_velocity_planner
{
OcclusionSpotInPrivateModule::OcclusionSpotInPrivateModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher)
: SceneModuleInterface(module_id, logger, clock),
  publisher_(publisher)
{
  param_ = planner_param;
}

bool OcclusionSpotInPrivateModule::modifyPathVelocity(
  autoware_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] autoware_planning_msgs::msg::StopReason * stop_reason)
{
  if (path->points.size() < 2) {return true;}
  param_.vehicle_info.baselink_to_front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  param_.vehicle_info.vehicle_width = planner_data_->vehicle_info_.vehicle_width_m;

  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  const double ego_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto & lanelet_map_ptr = planner_data_->lanelet_map;
  const auto & traffic_rules_ptr = planner_data_->traffic_rules;
  const auto & occ_grid_ptr = planner_data_->occupancy_grid;
  if (!lanelet_map_ptr || !traffic_rules_ptr || !occ_grid_ptr) {
    return true;
  }

  int first_idx = -1;
  int last_idx = -1;
  if (!planning_utils::calcClosestIndex<autoware_planning_msgs::msg::PathWithLaneId>(
      *path, ego_pose, first_idx, param_.dist_thr, param_.angle_thr))
  {
    return true;
  }
  const auto target_road_type = occlusion_spot_utils::ROAD_TYPE::PRIVATE;
  bool found_target = false;
  autoware_planning_msgs::msg::PathWithLaneId limited_path{};
  // search lanelet that includes target_road_type only : grantee last_idx >= 0 above
  for (size_t i = first_idx; i < path->points.size(); ++i) {
    occlusion_spot_utils::ROAD_TYPE search_road_type = occlusion_spot_utils::getCurrentRoadType(
      lanelet_map_ptr->laneletLayer.get(path->points[i].lane_ids[0]), lanelet_map_ptr);
    if (found_target && search_road_type != target_road_type) {break;}
    if (search_road_type == target_road_type) {
      last_idx = i;
      limited_path.points.emplace_back(path->points[i]);
      found_target = true;
    }
  }
  // this module use spline interpolation that needs more than 4 points
  if (limited_path.points.size() < 4) {return true;}
  nav_msgs::msg::OccupancyGrid occ_grid = *occ_grid_ptr;
  grid_map::GridMap grid_map;
  grid_utils::denoiseOccupancyGridCV(occ_grid, grid_map, param_.grid);
  if (param_.show_debug_grid) {publisher_->publish(occ_grid);}
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> possible_collisions;
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, *clock_, 3000,
    "first idx : " << first_idx << " last idx:" << last_idx);
  occlusion_spot_utils::generatePossibleCollisions(
    possible_collisions, limited_path, grid_map, first_idx, param_, debug_data_.sidewalks);
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, *clock_, 3000,
    "num possible collision:" << possible_collisions.size());
  occlusion_spot_utils::calcVelocityAndHeightToPossibleCollision(
    *path, possible_collisions);
  // apply safe velocity using ebs and pbs deceleration
  applySafeVelocityConsideringPossibleCollison(path, possible_collisions, ego_velocity, param_);
  debug_data_.z = path->points.front().point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;
  return true;
}

}  // namespace behavior_velocity_planner
