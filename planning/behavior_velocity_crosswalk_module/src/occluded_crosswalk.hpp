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
bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);

  constexpr double min_size = 0.5;  // TODO(Maxime): param
  const auto min_nb_of_cells = std::ceil(min_size / grid_map.getResolution());
  grid_map::Polygon incoming_area_poly;
  for (const auto & p : crosswalk_lanelet.polygon2d().basicPolygon())
    incoming_area_poly.addVertex(grid_map::Position(p.x(), p.y()));
  for (grid_map_utils::PolygonIterator iter(grid_map, incoming_area_poly); !iter.isPastEnd();
       ++iter) {
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
    if (is_occluded) return true;
  }
  return false;
}
}  // namespace behavior_velocity_planner

#endif  // OCCLUDED_CROSSWALK_HPP_
