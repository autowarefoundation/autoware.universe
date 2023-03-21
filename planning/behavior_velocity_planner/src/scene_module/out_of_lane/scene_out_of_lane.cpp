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

#include "scene_module/out_of_lane/scene_out_of_lane.hpp"

#include "scene_module/out_of_lane/decisions.hpp"
#include "scene_module/out_of_lane/footprint.hpp"
#include "scene_module/out_of_lane/out_of_lane_utils.hpp"
#include "scene_module/out_of_lane/overlapping_range.hpp"
#include "scene_module/out_of_lane/types.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

OutOfLaneModule::OutOfLaneModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  PlannerParam planner_param, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  params_(std::move(planner_param)),
  planner_data_(&planner_data)
{
}

bool OutOfLaneModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  std::cout << "modifyPathVelocity\n";
  debug_data_.reset_data();
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  out_of_lane_utils::EgoInfo ego_info;
  ego_info.pose = (*planner_data_)->current_odometry->pose;
  ego_info.path = *path;
  ego_info.first_path_idx =
    motion_utils::findNearestSegmentIndex(path->points, ego_info.pose.position);
  ego_info.velocity = (*planner_data_)->current_velocity->twist.linear.x;
  stopwatch.tic("calculate_path_footprints");
  const auto path_footprints = calculate_path_footprints(ego_info, params_);
  const auto calculate_path_footprints_us = stopwatch.toc("calculate_path_footprints");
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
  if (params_.skip_if_already_overlapping) {  // TODO(Maxime): cleanup
    const auto ego_footprint = calculate_current_ego_footprint(ego_info, params_);
    if (std::find_if(lanelets_to_check.begin(), lanelets_to_check.end(), [&](const auto & ll) {
          return boost::geometry::intersects(ll.polygon2d().basicPolygon(), ego_footprint);
        }) != lanelets_to_check.end()) {
      std::printf("*** ALREADY OVERLAPPING *** skipping\n");
    }
  }
  // Calculate overlapping ranges
  stopwatch.tic("calculate_overlapping_ranges");
  const auto ranges =
    calculate_overlapping_ranges(path_footprints, path_lanelets, lanelets_to_check, params_);
  const auto calculate_overlapping_ranges_us = stopwatch.toc("calculate_overlapping_ranges");
  std::printf("Found %lu ranges\n", ranges.size());
  for (const auto & i : ranges)
    std::printf("\t [%lu -> %lu] %ld\n", i.entering_path_idx, i.exiting_path_idx, i.lane.id());
  // Calculate stop and slowdown points
  std::printf("%lu detected objects\n", (*planner_data_)->predicted_objects->objects.size());
  stopwatch.tic("calculate_decisions");
  const auto decisions = calculate_decisions(
    ranges, ego_info, *(*planner_data_)->predicted_objects, (*planner_data_)->route_handler_,
    lanelets_to_check, params_, debug_data_);
  const auto calculate_decisions_us = stopwatch.toc("calculate_decisions");
  std::printf("Found %lu decisions\n", decisions.size());
  stopwatch.tic("insert_slowdown_points");
  insert_slowdown_points(*path, decisions, params_);
  const auto insert_slowdown_points_us = stopwatch.toc("insert_slowdown_points");
  for (const auto & decision : decisions) {
    // std::printf("\t [%lu] v = %2.2f m/s\n", decision.target_path_idx, decision.velocity);
    debug_data_.slowdown_poses.push_back(path->points[decision.target_path_idx].point.pose);
  }

  debug_data_.footprints = path_footprints;
  const auto total_time_us = stopwatch.toc();
  std::printf(
    "Total time = %2.2fus\n"
    "\tcalculate_path_footprints = %2.2fus\n"
    "\tcalculate_overlapping_ranges = %2.2fus\n"
    "\tcalculate_decisions = %2.2fus\n"
    "\tinsert_slowdown_points = %2.2fus\n",
    total_time_us, calculate_path_footprints_us, calculate_overlapping_ranges_us,
    calculate_decisions_us, insert_slowdown_points_us);
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

  // time ranges TODO(Maxime): remove
  debug_marker.id = 0;
  constexpr auto t_scale = 3.0;
  constexpr auto x_off = 3.0;
  constexpr auto y_off = -2.0;
  for (auto i = 0UL; i < debug_data_.ranges.size(); ++i) {
    // Enter/Exit points
    debug_marker.ns = "ranges";
    const auto & entering_p = debug_data_.ranges[i].entering_point;
    const auto & exiting_p = debug_data_.ranges[i].exiting_point;
    debug_marker.type = Marker::ARROW;
    debug_marker.scale.x = 0.2;
    debug_marker.scale.y = 0.2;
    debug_marker.scale.z = 0.2;
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.1, 1.0, 0.1, 0.5);
    debug_marker.points.push_back(
      tier4_autoware_utils::createMarkerPosition(entering_p.x(), entering_p.y(), 0));
    debug_marker.points.push_back(
      tier4_autoware_utils::createMarkerPosition(exiting_p.x(), exiting_p.y(), 0));
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;

    // t=0 point
    debug_marker.ns = "time ranges";
    auto p = debug_data_.ranges[i].entering_point;
    p.x() += x_off;
    p.y() += y_off;
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 1.0);
    debug_marker.points = {
      tier4_autoware_utils::createMarkerPosition(p.x(), p.y() - 3.0, 0),
      tier4_autoware_utils::createMarkerPosition(p.x(), p.y() + 3.0, 0)};
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    // time ranges of ego and npcs
    debug_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0, 0, 0);
    const auto & ego = debug_data_.ego_times[i];
    const auto & npcs = debug_data_.npc_times[i];
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.0, 0.0, 1.0, 0.7);
    debug_marker.scale.x = 3.0;
    debug_marker.scale.y = 3.0;
    debug_marker.scale.z = 1.0;
    debug_marker.points = {
      tier4_autoware_utils::createMarkerPosition(p.x() + t_scale * ego.first, p.y(), 0),
      tier4_autoware_utils::createMarkerPosition(p.x() + t_scale * ego.second, p.y(), 0)};
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    auto c = 0.25F;
    for (const auto & npc : npcs) {
      debug_marker.points = {
        tier4_autoware_utils::createMarkerPosition(p.x() + t_scale * npc.first, p.y(), 0),
        tier4_autoware_utils::createMarkerPosition(p.x() + t_scale * npc.second, p.y(), 0)};
      debug_marker.color = tier4_autoware_utils::createMarkerColor(c, c, c / 4, 0.7);
      debug_marker_array.markers.push_back(debug_marker);
      debug_marker.id++;
      c += 0.25;
    }
  }

  return debug_marker_array;
}

MarkerArray OutOfLaneModule::createVirtualWallMarkerArray()
{
  const auto current_time = this->clock_->now();

  MarkerArray wall_marker;
  std::string module_name = "out_of_lane";
  std::vector<Pose> slow_down_poses;
  for (const auto & slowdown_pose : debug_data_.slowdown_poses) {
    const auto p_front = calcOffsetPose(slowdown_pose, params_.front_offset, 0.0, 0.0);
    slow_down_poses.push_back(p_front);
    auto markers = virtual_wall_marker_creator_->createSlowDownVirtualWallMarker(
      slow_down_poses, module_name, current_time, static_cast<int32_t>(module_id_));
    for (auto & m : markers.markers) {
      m.id += static_cast<int>(wall_marker.markers.size());
      wall_marker.markers.push_back(std::move(m));
    }
  }
  return wall_marker;
}

}  // namespace behavior_velocity_planner
