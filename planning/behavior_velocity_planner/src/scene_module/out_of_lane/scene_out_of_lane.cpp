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
#include "scene_module/out_of_lane/overlapping_range.hpp"
#include "scene_module/out_of_lane/types.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

OutOfLaneModule::OutOfLaneModule(
  const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), params_(std::move(planner_param))
{
}

bool OutOfLaneModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  debug_data_.reset_data();
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  out_of_lane_utils::EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path = path;
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(path->points, ego_data.pose.position);
  ego_data.velocity = planner_data_->current_velocity->twist.linear.x;
  ego_data.max_decel = -planner_data_->max_stop_acceleration_threshold;
  stopwatch.tic("calculate_path_footprints");
  const auto current_ego_footprint = calculate_current_ego_footprint(ego_data, params_, true);
  const auto path_footprints = calculate_path_footprints(ego_data, params_);
  const auto calculate_path_footprints_us = stopwatch.toc("calculate_path_footprints");
  // Get path lanelets, other lanelets, and lanelets to ignore TODO(Maxime): make separate function
  const auto contains_lanelet = [](const auto & container, const auto & ll) -> bool {
    return std::find_if(container.begin(), container.end(), [&](const auto & l) {
             return l.id() == ll.id();
           }) != container.end();
  };
  const auto path_lanelets = planning_utils::getLaneletsOnPath(
    *path, planner_data_->route_handler_->getLaneletMapPtr(),
    planner_data_->current_odometry->pose);
  lanelet::ConstLanelets lanelets_to_ignore;
  for (const auto & path_lanelet : path_lanelets) {
    for (const auto & following :
         planner_data_->route_handler_->getRoutingGraphPtr()->following(path_lanelet)) {
      if (!contains_lanelet(path_lanelets, following)) {
        lanelets_to_ignore.push_back(following);
      }
    }
  }
  const auto behind =
    planning_utils::calculateOffsetPoint2d(ego_data.pose, params_.rear_offset, 0.0);
  const lanelet::BasicPoint2d behind_point(behind.x(), behind.y());
  const auto behind_lanelets = lanelet::geometry::findWithin2d(
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer, behind_point, 0.0);
  for (const auto & l : behind_lanelets) {
    const auto is_path_lanelet = contains_lanelet(path_lanelets, l.second);
    if (!is_path_lanelet) lanelets_to_ignore.push_back(l.second);
  }
  const lanelet::BasicPoint2d ego_point(ego_data.pose.position.x, ego_data.pose.position.y);
  const auto lanelets_within_range = lanelet::geometry::findWithin2d(
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer, ego_point,
    std::max(params_.slow_dist_threshold, params_.stop_dist_threshold));
  const auto other_lanelets = [&]() {
    lanelet::ConstLanelets lls;
    for (const auto & ll : lanelets_within_range) {
      const auto is_path_lanelet = contains_lanelet(path_lanelets, ll.second);
      const auto is_overlapped_by_path_lanelets = [&](const auto & l) {
        for (const auto & path_ll : path_lanelets)
          if (boost::geometry::overlaps(
                path_ll.polygon2d().basicPolygon(), l.polygon2d().basicPolygon()))
            return true;
        return false;
      };
      if (!is_path_lanelet && !is_overlapped_by_path_lanelets(ll.second)) lls.push_back(ll.second);
    }
    return lls;
  }();
  debug_data_.path_lanelets = path_lanelets;
  debug_data_.ignored_lanelets = lanelets_to_ignore;
  debug_data_.other_lanelets = other_lanelets;
  if (params_.skip_if_already_overlapping) {  // TODO(Maxime): cleanup
    debug_data_.current_footprint = current_ego_footprint;
    const auto overlapped_lanelet_it =
      std::find_if(other_lanelets.begin(), other_lanelets.end(), [&](const auto & ll) {
        return boost::geometry::intersects(ll.polygon2d().basicPolygon(), current_ego_footprint);
      });
    if (overlapped_lanelet_it != other_lanelets.end()) {
      debug_data_.current_overlapped_lanelets.push_back(*overlapped_lanelet_it);
      std::printf("*** ALREADY OVERLAPPING *** skipping\n");
      return true;
    }
  }
  // Calculate overlapping ranges
  stopwatch.tic("calculate_overlapping_ranges");
  const auto ranges =
    calculate_overlapping_ranges(path_footprints, path_lanelets, other_lanelets, params_);
  const auto calculate_overlapping_ranges_us = stopwatch.toc("calculate_overlapping_ranges");
  // Calculate stop and slowdown points
  stopwatch.tic("calculate_decisions");
  auto decisions = calculate_decisions(
    ranges, ego_data, *planner_data_->predicted_objects, planner_data_->route_handler_,
    other_lanelets, params_, debug_data_);
  const auto calculate_decisions_us = stopwatch.toc("calculate_decisions");
  stopwatch.tic("calc_slowdown_points");
  const auto points_to_insert = calculate_slowdown_points(ego_data, decisions, params_);
  const auto calc_slowdown_points_us = stopwatch.toc("calc_slowdown_points");
  stopwatch.tic("insert_slowdown_points");
  for (const auto & point : points_to_insert) {
    auto path_idx = point.slowdown.target_path_idx;
    planning_utils::insertVelocity(*ego_data.path, point.point, point.slowdown.velocity, path_idx);
  }
  const auto insert_slowdown_points_us = stopwatch.toc("insert_slowdown_points");

  debug_data_.footprints = path_footprints;
  debug_data_.slowdowns = points_to_insert;
  const auto total_time_us = stopwatch.toc();
  std::printf(
    "Total time = %2.2fus\n"
    "\tcalculate_path_footprints = %2.0fus\n"
    "\tcalculate_overlapping_ranges = %2.0fus\n"
    "\tcalculate_decisions = %2.0fus\n"
    "\tcalc_slowdown_points = %2.0fus\n"
    "\tinsert_slowdown_points = %2.0fus\n",
    total_time_us, calculate_path_footprints_us, calculate_overlapping_ranges_us,
    calculate_decisions_us, calc_slowdown_points_us, insert_slowdown_points_us);
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

  // current footprint + overlapped lanelets (if any)
  debug_marker.ns = "current_overlap";
  debug_marker.points.clear();
  for (const auto & p : debug_data_.current_footprint)
    debug_marker.points.push_back(
      tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
  debug_marker.points.push_back(debug_marker.points.front());
  if (debug_data_.current_overlapped_lanelets.empty())
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.1, 1.0, 0.1, 0.5);
  else
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  debug_marker_array.markers.push_back(debug_marker);
  debug_marker.id++;
  for (const auto & ll : debug_data_.current_overlapped_lanelets) {
    debug_marker.points.clear();
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }

  // Lanelets
  debug_marker.ns = "path_lanelets";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.1, 0.1, 1.0, 0.5);
  for (const auto & ll : debug_data_.path_lanelets) {
    debug_marker.points.clear();
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  debug_marker.ns = "ignored_lanelets";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.7, 0.7, 0.2, 0.5);
  for (const auto & ll : debug_data_.ignored_lanelets) {
    debug_marker.points.clear();
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  debug_marker.ns = "other_lanelets";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.4, 0.4, 0.7, 0.5);
  for (const auto & ll : debug_data_.other_lanelets) {
    debug_marker.points.clear();
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  return debug_marker_array;
}

MarkerArray OutOfLaneModule::createVirtualWallMarkerArray()
{
  const auto current_time = this->clock_->now();

  MarkerArray wall_marker;
  std::string module_name = "out_of_lane";
  std::vector<Pose> slow_down_poses;
  for (const auto & slowdown : debug_data_.slowdowns) {
    const auto p_front = calcOffsetPose(slowdown.point.point.pose, params_.front_offset, 0.0, 0.0);
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
