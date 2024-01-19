// Copyright 2024 Tier IV, Inc.
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

#include "occluded_crosswalk.hpp"

#include "behavior_velocity_crosswalk_module/util.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>

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
bool is_occluded(
  const grid_map::GridMap & grid_map, const int min_nb_of_cells, const grid_map::Index idx,
  const behavior_velocity_planner::CrosswalkModule::PlannerParam & params)
{
  grid_map::Index idx_offset;
  for (idx_offset.x() = 0; idx_offset.x() < min_nb_of_cells; ++idx_offset.x()) {
    for (idx_offset.y() = 0; idx_offset.y() < min_nb_of_cells; ++idx_offset.y()) {
      const auto index = idx + idx_offset;
      if ((index < grid_map.getSize()).all()) {
        const auto cell_value = grid_map.at("layer", index);
        if (
          cell_value < params.occlusion_free_space_max ||
          cell_value > params.occlusion_occupied_min)
          return false;
      }
    }
  }
  return true;
}

lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicSegment2d & segment, const double extra_distance)
{
  const auto direction_vector = (segment.second - segment.first).normalized();
  return segment.second + extra_distance * direction_vector;
}

bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Point & path_intersection,
  const behavior_velocity_planner::CrosswalkModule::PlannerParam & params)
{
  // Declare a stream and an SVG mapper
  // std::ofstream svg("/home/mclement/Pictures/crosswalk.svg");  // /!\ CHANGE PATH
  // boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 400, 400);
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
  const lanelet::BasicPoint2d path_inter(path_intersection.x, path_intersection.y);

  const auto min_nb_of_cells = std::ceil(params.occlusion_min_size / grid_map.getResolution());
  std::vector<lanelet::BasicPolygon2d> incoming_areas;
  const std::vector<std::function<lanelet::BasicSegment2d(lanelet::ConstLineString2d)>>
    segment_getters = {
      [](const auto & ls) -> lanelet::BasicSegment2d {
        return {ls[1].basicPoint2d(), ls[0].basicPoint2d()};
      },
      [](const auto & ls) -> lanelet::BasicSegment2d {
        const auto size = ls.size();
        return {ls[size - 2].basicPoint2d(), ls[size - 1].basicPoint2d()};
      }};
  if (
    crosswalk_lanelet.centerline2d().size() > 1 && crosswalk_lanelet.leftBound2d().size() > 1 &&
    crosswalk_lanelet.rightBound2d().size() > 1) {
    for (const auto segment_getter : segment_getters) {
      const auto center_segment = segment_getter(crosswalk_lanelet.centerline2d());
      const auto left_segment = segment_getter(crosswalk_lanelet.leftBound2d());
      const auto right_segment = segment_getter(crosswalk_lanelet.rightBound2d());
      const auto dist = lanelet::geometry::distance2d(center_segment.second, path_inter);
      if (dist < params.occlusion_detection_range) {
        const auto target_left =
          interpolate_point(left_segment, params.occlusion_detection_range - dist);
        const auto target_right =
          interpolate_point(right_segment, params.occlusion_detection_range - dist);
        incoming_areas.push_back(
          {left_segment.second, target_left, target_right, right_segment.second});
      }
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
      if (is_occluded(grid_map, min_nb_of_cells, *iter, params)) return true;
  }
  return false;
}
}  // namespace behavior_velocity_planner
