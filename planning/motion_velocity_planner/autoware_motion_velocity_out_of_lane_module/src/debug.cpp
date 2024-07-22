// Copyright 2024 TIER IV, Inc.
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

#include "debug.hpp"

#include "types.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <lanelet2_core/Forward.h>

#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane::debug
{
namespace
{

visualization_msgs::msg::Marker get_base_marker()
{
  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = "map";
  base_marker.header.stamp = rclcpp::Time(0);
  base_marker.id = 0;
  base_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  base_marker.action = visualization_msgs::msg::Marker::ADD;
  base_marker.pose.position = universe_utils::createMarkerPosition(0.0, 0.0, 0);
  base_marker.pose.orientation = universe_utils::createMarkerOrientation(0, 0, 0, 1.0);
  base_marker.scale = universe_utils::createMarkerScale(0.1, 0.1, 0.1);
  base_marker.color = universe_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  return base_marker;
}
void add_polygons_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygons2d & polygons, const double z, const size_t prev_nb,
  const std::string & ns)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = ns;
  for (const auto & f : polygons) {
    debug_marker.points.clear();
    for (const auto & p : f)
      debug_marker.points.push_back(universe_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  debug_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id) {
    debug_marker_array.markers.push_back(debug_marker);
  }
}

void add_out_of_lane_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array, const OutOfLaneData & data,
  const double z, DebugData & debug_data)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = "out_of_lane_areas";
  for (const auto & p : data.outside_points) {
    if (p.to_avoid) {
      debug_marker.color.r = 1.0;
      debug_marker.color.g = 0.0;
    } else {
      debug_marker.color.r = 0.0;
      debug_marker.color.g = 1.0;
    }
    for (const auto & f : p.outside_rings) {
      debug_marker.points.clear();
      for (const auto & p : f)
        debug_marker.points.push_back(universe_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
      debug_marker.points.push_back(debug_marker.points.front());
      debug_marker_array.markers.push_back(debug_marker);
      debug_marker.id++;
    }
  }
  size_t nb_markers = debug_marker.id;
  debug_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (; debug_marker.id < static_cast<int>(debug_data.prev_out_of_lane_areas); ++debug_marker.id) {
    debug_marker_array.markers.push_back(debug_marker);
  }
  debug_data.prev_out_of_lane_areas = nb_markers;
}

void add_current_overlap_marker(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygon2d & current_footprint, const double z)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = "current_overlap";
  debug_marker.points.clear();
  for (const auto & p : current_footprint)
    debug_marker.points.push_back(universe_utils::createMarkerPosition(p.x(), p.y(), z));
  if (!debug_marker.points.empty()) debug_marker.points.push_back(debug_marker.points.front());
  debug_marker.color = universe_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  debug_marker_array.markers.push_back(debug_marker);
  debug_marker.id++;
}

void add_ttc_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array, const EgoData & ego_data,
  const OutOfLaneData & out_of_lane_data, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  debug_marker.scale.z = 0.5;
  debug_marker.ns = "ttcs";
  for (const auto & p : out_of_lane_data.outside_points) {
    if (p.to_avoid) {
      debug_marker.color.r = 1.0;
      debug_marker.color.g = 0.0;
    } else {
      debug_marker.color.r = 0.0;
      debug_marker.color.g = 1.0;
    }
    if (p.ttc) {
      debug_marker.pose = ego_data.trajectory_points[p.trajectory_index].pose;
      debug_marker.text = std::to_string(*p.ttc);
      debug_marker_array.markers.push_back(debug_marker);
      debug_marker.id++;
    }
  }
  debug_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id) {
    debug_marker_array.markers.push_back(debug_marker);
  }
}

void add_lanelet_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = ns;
  debug_marker.color = color;
  for (const auto & ll : lanelets) {
    debug_marker.points.clear();

    // add a small z offset to draw above the lanelet map
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        universe_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.1));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  debug_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
}
}  // namespace

visualization_msgs::msg::MarkerArray create_debug_marker_array(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data, DebugData & debug_data)
{
  constexpr auto z = 0.0;
  visualization_msgs::msg::MarkerArray debug_marker_array;

  add_polygons_markers(
    debug_marker_array, ego_data.trajectory_footprints, z, debug_data.prev_footprints,
    "footprints");
  debug_data.prev_footprints = ego_data.trajectory_footprints.size();

  add_lanelet_markers(
    debug_marker_array, ego_data.route_lanelets, "route_lanelets",
    universe_utils::createMarkerColor(0.1, 0.1, 1.0, 0.5), debug_data.prev_route_lanelets);
  debug_data.prev_route_lanelets = ego_data.route_lanelets.size();

  add_lanelet_markers(
    debug_marker_array, ego_data.ignored_lanelets, "ignored_lanelets",
    universe_utils::createMarkerColor(0.7, 0.7, 0.2, 0.5), debug_data.prev_ignored_lanelets);
  debug_data.prev_ignored_lanelets = ego_data.ignored_lanelets.size();

  add_lanelet_markers(
    debug_marker_array, out_of_lane_data.out_of_lane_lanelets, "out_of_lane_lanelets",
    universe_utils::createMarkerColor(0.4, 0.4, 0.7, 0.5), debug_data.prev_out_of_lane_lanelets);
  debug_data.prev_out_of_lane_lanelets = out_of_lane_data.out_of_lane_lanelets.size();

  lanelet::BasicPolygons2d drivable_lane_polygons;
  for (const auto & poly : ego_data.drivable_lane_polygons) {
    drivable_lane_polygons.push_back(poly.outer);
  }
  add_polygons_markers(
    debug_marker_array, drivable_lane_polygons, z, debug_data.prev_drivable_lane_polygons,
    "ego_lane");
  debug_data.prev_drivable_lane_polygons = drivable_lane_polygons.size();

  lanelet::BasicPolygons2d out_of_lane_areas;
  for (const auto & p : out_of_lane_data.outside_points) {
    out_of_lane_areas.insert(
      out_of_lane_areas.end(), p.outside_rings.begin(), p.outside_rings.end());
  }

  add_out_of_lane_markers(debug_marker_array, out_of_lane_data, z, debug_data);

  add_current_overlap_marker(debug_marker_array, ego_data.current_footprint, z);

  add_ttc_markers(debug_marker_array, ego_data, out_of_lane_data, debug_data.prev_ttcs);
  debug_data.prev_ttcs = out_of_lane_data.outside_points.size();

  return debug_marker_array;
}

motion_utils::VirtualWalls create_virtual_walls(
  const geometry_msgs::msg::Pose & pose, const bool stop, const PlannerParam & params)
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "out_of_lane";
  wall.longitudinal_offset = params.front_offset;
  wall.style = stop ? motion_utils::VirtualWallType::stop : motion_utils::VirtualWallType::slowdown;
  wall.pose = pose;
  virtual_walls.push_back(wall);
  return virtual_walls;
}
}  // namespace autoware::motion_velocity_planner::out_of_lane::debug
