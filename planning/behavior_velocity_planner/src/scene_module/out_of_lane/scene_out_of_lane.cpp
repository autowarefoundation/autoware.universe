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
: SceneModuleInterface(module_id, logger, clock), param_(planner_param), planner_data_(planner_data)
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
  // TODO(Maxime): How about caching since the path do not change often which means we do not need
  // to recalculate the footprints/overlaps Calculate path point footprints
  const auto path_footprints = calculate_path_footprints(*path, param_);
  // Get neighboring lanes TODO(Maxime): make separate function
  const auto path_lanelets = planning_utils::getLaneletsOnPath(
    *path, planner_data_->route_handler_->getLaneletMapPtr(),
    planner_data_->current_odometry->pose);
  // TODO(Maxime): naively get ALL lanelets that are not the path lanelets. Probably too expensive
  // for production.
  const auto lanelets_to_check = [&]() {
    lanelet::ConstLanelets lls;
    for (const auto & ll : planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer) {
      if (std::find_if(path_lanelets.begin(), path_lanelets.end(), [&](const auto & l) {
            return l.id() == ll.id();
          }) == path_lanelets.end())
        lls.push_back(ll);
    }
    return lls;
  }();
  // Calculate overlaps
  const auto overlaps = calculate_overlaps(path_footprints, lanelets_to_check, param_);
  planner_data_->route_handler_->getLaneletsFromIds(overlaps.front().overlapped_lanes);
  // Calculate intervals
  // Calculate stop and slowdown points

  // TODO(Maxime): don't copy data.
  debug_data_.footprints = path_footprints;
  debug_data_.overlaps = overlaps;
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
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }

  debug_marker.ns = "overlaps";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.0, 0.1, 1.0, 0.5);
  for (const auto & overlap : debug_data_.overlaps) {
    for (const auto & ll_id : overlap.overlapped_lanes) {
      const auto ll_poly = planner_data_->route_handler_->getLaneletMapPtr()
                             ->laneletLayer.get(ll_id)
                             .polygon2d()
                             .basicPolygon();
      lanelet::BasicPolygons2d intersections;
      boost::geometry::intersection(
        ll_poly, debug_data_.footprints[overlap.path_idx], intersections);
      debug_marker.points.clear();
      for (const auto & intersection : intersections)
        for (const auto & p : intersection)
          debug_marker.points.push_back(
            tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
      debug_marker.id++;
      debug_marker_array.markers.push_back(debug_marker);
    }
  }
  return debug_marker_array;
}

MarkerArray OutOfLaneModule::createVirtualWallMarkerArray()
{
  MarkerArray wall_marker;
  return wall_marker;
}

}  // namespace behavior_velocity_planner
