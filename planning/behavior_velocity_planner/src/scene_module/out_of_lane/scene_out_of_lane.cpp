// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/out_of_lane/out_of_lane_utils.hpp>
#include <scene_module/out_of_lane/scene_out_of_lane.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/debug.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/trajectory_utils.hpp>
#include <utilization/util.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

namespace behavior_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

OutOfLaneModule::OutOfLaneModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  params_(planner_param),
  planner_data_(&planner_data)
{
}

bool OutOfLaneModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  std::cout << "modifyPathVelocity\n";
  debug_data_.resetData();
  if (!path || path->points.size() < 2) {
    return true;
  }
  const auto ego_idx =
    motion_utils::findNearestIndex(path->points, (*planner_data_)->current_odometry->pose.position);
  // const auto ego_vel = (*planner_data_)->current_velocity->twist.linear.x;
  const auto path_footprints = calculate_path_footprints(*path, ego_idx, params_);
  // Get neighboring lanes TODO(Maxime): make separate function
  const auto path_lanelets = planning_utils::getLaneletsOnPath(
    *path, (*planner_data_)->route_handler_->getLaneletMapPtr(),
    (*planner_data_)->current_odometry->pose);
  // TODO(Maxime): naively get ALL lanelets that are not the path lanelets. Probably too expensive
  const auto lanelets_to_check = [&]() {
    lanelet::ConstLanelets lls;
    for (const auto & ll : (*planner_data_)->route_handler_->getLaneletMapPtr()->laneletLayer) {
      if (std::find_if(path_lanelets.begin(), path_lanelets.end(), [&](const auto & l) {
            return l.id() == ll.id();
          }) == path_lanelets.end())
        lls.push_back(ll);
    }
    return lls;
  }();
  // Calculate overlapping intervals
  const auto intervals = calculate_overlapping_intervals(
    path_footprints, path_lanelets, ego_idx, lanelets_to_check, params_);
  std::printf("Found %lu intervals\n", intervals.size());
  for (const auto & i : intervals)
    std::printf("\t [%lu -> %lu] %ld\n", i.entering_path_idx, i.exiting_path_idx, i.lane.id());
  // Calculate stop and slowdown points
  std::printf("%lu detected objects\n", (*planner_data_)->predicted_objects->objects.size());
  const auto decisions = calculate_decisions(
    intervals, *path, ego_idx, *(*planner_data_)->predicted_objects,
    (*planner_data_)->route_handler_, lanelets_to_check);
  std::printf("Found %lu decisions\n", decisions.size());
  insert_slowdown_points(*path, decisions, params_);
  for (const auto & decision : decisions) {
    // std::printf("\t [%lu] v = %2.2f m/s\n", decision.target_path_idx, decision.velocity);
    // TODO(Maxime): properly insert stop point
    path->points[decision.target_path_idx].point.longitudinal_velocity_mps = decision.velocity;
    debug_data_.slowdown_poses.push_back(path->points[decision.target_path_idx].point.pose);
  }

  debug_data_.footprints = std::move(path_footprints);
  return true;
}

MarkerArray OutOfLaneModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = 0;
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0);
  debug_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1);
  debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  debug_marker.ns = "footprints";
  for (const auto & f : debug_data_.footprints) {
    debug_marker.points.clear();
    for (const auto & p : f)
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_marker_array;
}

MarkerArray OutOfLaneModule::createVirtualWallMarkerArray()
{
  const auto current_time = this->clock_->now();

  MarkerArray wall_marker;
  std::string module_name = "out_of_lane";
  std::vector<Pose> slow_down_poses;
  for (auto i = 0UL; i < debug_data_.slowdown_poses.size(); i++) {
    const auto p_front =
      calcOffsetPose(debug_data_.slowdown_poses[i], params_.front_offset, 0.0, 0.0);
    slow_down_poses.push_back(p_front);
    auto markers = virtual_wall_marker_creator_->createSlowDownVirtualWallMarker(
      slow_down_poses, module_name, current_time, module_id_);
    for (auto & m : markers.markers) {
      m.id += wall_marker.markers.size();
      wall_marker.markers.push_back(std::move(m));
    }
  }
  return wall_marker;
}

}  // namespace behavior_velocity_planner
