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

#ifndef OCCLUDED_OBJECT_HPP_
#define OCCLUDED_OBJECT_HPP_

#include "behavior_velocity_crosswalk_module/util.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>
#include <interpolation/linear_interpolation.hpp>
#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <limits>
#include <optional>
#include <vector>

namespace behavior_velocity_planner
{
std::optional<autoware_auto_perception_msgs::msg::PredictedObject> create_occluded_object(
  const tier4_autoware_utils::Polygon2d & attention_polygon,
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & ego_path,
  const geometry_msgs::msg::Point & ego_position)
{
  // TODO(Maxime): param
  constexpr auto virtual_object_size = 0.5;
  constexpr auto virtual_object_velocity = 1.0;
  constexpr auto virtual_object_time_resolution = 0.5;
  constexpr auto virtual_object_time_horizon = 5.0;

  std::optional<autoware_auto_perception_msgs::msg::PredictedObject> occluded_object{};

  tier4_autoware_utils::LineString2d path_ls;
  for (const auto & p : ego_path.points)
    path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);

  tier4_autoware_utils::MultiPoint2d intersections;
  tier4_autoware_utils::LineString2d crosswalk_centerline;
  for (const auto & p : crosswalk_lanelet.centerline2d().basicLineString())
    crosswalk_centerline.emplace_back(p.x(), p.y());
  boost::geometry::intersection(crosswalk_centerline, path_ls, intersections);
  if (intersections.empty()) return {};
  const auto crosswalk_center = intersections.front();
  const auto range_distance = virtual_object_velocity * virtual_object_time_horizon;
  const grid_map::Polygon in_range_poly{std::vector<grid_map::Position>({
    {crosswalk_center.x() - range_distance, crosswalk_center.y() + range_distance},
    {crosswalk_center.x() + range_distance, crosswalk_center.y() + range_distance},
    {crosswalk_center.x() + range_distance, crosswalk_center.y() - range_distance},
    {crosswalk_center.x() - range_distance, crosswalk_center.y() - range_distance},
  })};
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  // ignore the attention polygon
  grid_map::Polygon attention_poly;
  boost::geometry::for_each_point(
    attention_polygon, [&](const auto & p) { attention_poly.addVertex(p); });
  for (auto iter = grid_map_utils::PolygonIterator(grid_map, attention_poly); !iter.isPastEnd();
       ++iter)
    grid_map.at("layer", *iter) = 0.0;

  // find occluded point closest to the crosswalk center
  const auto crosswalk_center_arc_length =
    lanelet::geometry::toArcCoordinates(crosswalk_lanelet.centerline2d(), crosswalk_center).length;
  std::optional<lanelet::ArcCoordinates> closest_arc_coordinates;
  double closest_arc_length_distance = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point p;
  grid_map::Position closest_position;
  const auto min_nb_of_cells = std::ceil(virtual_object_size / grid_map.getResolution());
  for (grid_map_utils::PolygonIterator iter(grid_map, in_range_poly); !iter.isPastEnd(); ++iter) {
    const auto idx = *iter;
    // TODO(Maxime): move to function
    const auto is_occluded = [&]() {
      grid_map::Index idx_offset;
      for (idx_offset.x() = 0; idx_offset.x() < min_nb_of_cells; ++idx_offset.x()) {
        for (idx_offset.y() = 0; idx_offset.y() < min_nb_of_cells; ++idx_offset.y()) {
          const auto index = idx + idx_offset;
          if ((index < grid_map.getSize()).all()) {
            const auto cell_value = grid_map.at("layer", index);
            // TODO(Maxime): magic number -> params
            if (cell_value < 47 || cell_value > 53) return false;
          }
        }
      }
      return true;
    }();
    if (is_occluded) {
      grid_map::Position position;
      grid_map.getPosition(idx, position);
      // this projection can be outside of the centerline
      const auto projection =
        lanelet::geometry::project(crosswalk_lanelet.centerline2d(), position);
      // this arc coordinate is clamped between 0 and the length of the centerline
      auto arc_coordinates =
        lanelet::geometry::toArcCoordinates(crosswalk_lanelet.centerline2d(), projection);
      arc_coordinates.distance = lanelet::geometry::distance2d(projection, position);
      if (arc_coordinates.length <= 0.0) {
        arc_coordinates.length =
          -lanelet::geometry::distance2d(crosswalk_lanelet.centerline2d().front(), projection);
      } else if (
        arc_coordinates.length >= lanelet::geometry::length(crosswalk_lanelet.centerline2d())) {
        arc_coordinates.length =
          lanelet::geometry::length(crosswalk_lanelet.centerline2d()) +
          lanelet::geometry::distance2d(crosswalk_lanelet.centerline2d().back(), projection);
      }
      const auto dist = std::abs(arc_coordinates.length - crosswalk_center_arc_length);
      p.x = position.x();
      p.y = position.y();
      const auto is_after_ego =
        motion_utils::calcSignedArcLength(ego_path.points, ego_position, p) > 0.0;
      if (is_after_ego && dist < closest_arc_length_distance) {
        closest_arc_length_distance = dist;
        closest_arc_coordinates = arc_coordinates;
        closest_position = position;
      }
    }
  }
  // TODO(Maxime): REMOVE
  {
    tier4_autoware_utils::Polygon2d in_range_polygon;
    for (const auto & p : in_range_poly.getVertices())
      in_range_polygon.outer().emplace_back(p.x(), p.y());
    boost::geometry::correct(in_range_polygon);
  }
  if (closest_arc_coordinates) {
    const auto inverse_projected_point = lanelet::geometry::fromArcCoordinates(
      crosswalk_lanelet.centerline2d(), *closest_arc_coordinates);
    closest_arc_coordinates->distance = 0.0;  // assume a straight path to the crosswalk
    const auto initial_position = lanelet::geometry::fromArcCoordinates(
      crosswalk_lanelet.centerline2d(), *closest_arc_coordinates);
    occluded_object.emplace();
    occluded_object->kinematics.initial_pose_with_covariance.pose.position.x = initial_position.x();
    occluded_object->kinematics.initial_pose_with_covariance.pose.position.y = initial_position.y();
    occluded_object->kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
    occluded_object->shape.dimensions.x = virtual_object_size;
    occluded_object->shape.dimensions.y = virtual_object_size;
    occluded_object->classification.emplace_back();
    occluded_object->classification.back().probability = 1.0;
    occluded_object->classification.back().label =
      autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;

    occluded_object->kinematics.predicted_paths.emplace_back();
    occluded_object->kinematics.predicted_paths.back().confidence = 1.0;
    occluded_object->kinematics.predicted_paths.back().time_step =
      rclcpp::Duration::from_seconds(virtual_object_time_resolution);
    geometry_msgs::msg::Point crosswalk_center_position;
    crosswalk_center_position.x = crosswalk_center.x();
    crosswalk_center_position.y = crosswalk_center.y();
    const auto dist_to_target = tier4_autoware_utils::calcDistance2d(
      occluded_object->kinematics.initial_pose_with_covariance.pose.position,
      crosswalk_center_position);
    geometry_msgs::msg::Pose pose;
    for (auto s = virtual_object_time_resolution * virtual_object_velocity;
         s <= virtual_object_time_horizon * virtual_object_velocity;
         s += virtual_object_time_resolution * virtual_object_velocity) {
      pose.position.x = interpolation::lerp(
        occluded_object->kinematics.initial_pose_with_covariance.pose.position.x,
        crosswalk_center_position.x, s / dist_to_target);
      pose.position.y = interpolation::lerp(
        occluded_object->kinematics.initial_pose_with_covariance.pose.position.y,
        crosswalk_center_position.y, s / dist_to_target);
      occluded_object->kinematics.predicted_paths.back().path.push_back(pose);
    }
  }
  return occluded_object;
}
}  // namespace behavior_velocity_planner

#endif  // OCCLUDED_OBJECT_HPP_
