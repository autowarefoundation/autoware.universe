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

#ifndef OCCLUDED_CROSSWALK_HPP_
#define OCCLUDED_CROSSWALK_HPP_

#include "behavior_velocity_crosswalk_module/util.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/geometry/Point.h>

#include <vector>

/*
// for writing the svg file
#include <fstream>
#include <iostream>
// for the geometry types
#include <tier4_autoware_utils/geometry/geometry.hpp>
// for the svg mapper
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>
*/
namespace behavior_velocity_planner
{
/// @brief check if the gridmap is occluded at the given index
bool is_occluded(
  const grid_map::GridMap & grid_map, const int min_nb_of_cells, const grid_map::Index idx)
{
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
}

lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2, const double extra_distance)
{
  const auto direction_vector = (p2 - p1).normalized();
  return p2 + extra_distance * direction_vector;
}

/// @brief check if the crosswalk is occluded
bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Point & path_intersection)
{
  // Declare a stream and an SVG mapper
  // std::ofstream svg("/home/mclement/Pictures/crosswalk.svg");  // /!\ CHANGE PATH
  // boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 400, 400);
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  const lanelet::BasicPoint2d path_inter(path_intersection.x, path_intersection.y);

  constexpr double min_size = 0.5;  // TODO(Maxime): param
  constexpr double range = 8.0;     // TODO(Maxime): param, should include the ego width ?
  const auto min_nb_of_cells = std::ceil(min_size / grid_map.getResolution());
  std::vector<lanelet::BasicPolygon2d> incoming_areas;
  // front
  {
    const auto dist = lanelet::geometry::distance2d(
      crosswalk_lanelet.centerline2d().basicLineString().front(), path_inter);
    if (dist < range) {
      const auto & base_left = crosswalk_lanelet.leftBound2d().front().basicPoint2d();
      const auto & base_right = crosswalk_lanelet.rightBound2d().front().basicPoint2d();
      const auto target_left = interpolate_point(
        *(crosswalk_lanelet.leftBound2d().basicBegin() + 1), base_left, range - dist);
      const auto target_right = interpolate_point(
        *(crosswalk_lanelet.rightBound2d().basicBegin() + 1), base_right, range - dist);
      incoming_areas.push_back({base_left, target_left, target_right, base_right});
    }
  }
  // back
  {
    const auto dist = lanelet::geometry::distance2d(
      crosswalk_lanelet.centerline2d().basicLineString().back(), path_inter);
    if (dist < range) {
      const auto & base_left = crosswalk_lanelet.leftBound2d().back().basicPoint2d();
      const auto & base_right = crosswalk_lanelet.rightBound2d().back().basicPoint2d();
      const auto target_left = interpolate_point(
        *(crosswalk_lanelet.leftBound2d().basicEnd() - 2), base_left, range - dist);
      const auto target_right = interpolate_point(
        *(crosswalk_lanelet.rightBound2d().basicEnd() - 2), base_right, range - dist);
      incoming_areas.push_back({base_left, target_left, target_right, base_right});
    }
  }
  incoming_areas.push_back(crosswalk_lanelet.polygon2d().basicPolygon());
  // mapper.add(crosswalk_lanelet.polygon2d().basicPolygon());
  // mapper.add(path_inter);
  // mapper.add(incoming_areas[0]);
  // mapper.add(incoming_areas[1]);
  // mapper.map(
  //   crosswalk_lanelet.polygon2d().basicPolygon(),
  //   "fill-opacity:0.3;fill:grey;stroke:black;stroke-width:1");
  // mapper.map(path_inter, "opacity:0.5;fill:pink;stroke:pink;stroke-width:1", 1);
  // mapper.map(incoming_areas[0], "fill-opacity:0.3;fill:green;stroke:none;stroke-width:1");
  // mapper.map(incoming_areas[1], "fill-opacity:0.3;fill:red;stroke:none;stroke-width:1");
  for (const auto & incoming_area : incoming_areas) {
    grid_map::Polygon poly;
    for (const auto & p : incoming_area) poly.addVertex(grid_map::Position(p.x(), p.y()));
    for (grid_map_utils::PolygonIterator iter(grid_map, poly); !iter.isPastEnd(); ++iter)
      if (is_occluded(grid_map, min_nb_of_cells, *iter)) return true;
  }
  return false;
}
}  // namespace behavior_velocity_planner

#endif  // OCCLUDED_CROSSWALK_HPP_
