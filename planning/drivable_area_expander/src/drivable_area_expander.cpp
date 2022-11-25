// Copyright 2022 TIER IV, Inc.
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

#include "drivable_area_expander/drivable_area_expander.hpp"

#include "drivable_area_expander/grid_utils.hpp"
#include "drivable_area_expander/obstacles.hpp"
#include "drivable_area_expander/parameters.hpp"
#include "drivable_area_expander/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace drivable_area_expander
{
OccupancyGrid buildExpandedDrivableArea(
  const OccupancyGrid & drivable_area, const multipolygon_t & footprint,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const point_t & origin)
{
  const auto grid_map = convertToGridMap(drivable_area);
  const auto footprint_grid_map =
    makeFootprintGridMap(grid_map, footprint, predicted_paths, uncrossable_lines, origin);
  return convertToOccupancyGrid(footprint_grid_map);
}

multipolygon_t createPredictedPathPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  const ExpansionParameters & params)
{
  multipolygon_t predicted_path_polygons;
  if (params.avoid_dynamic_objects) {
    predicted_path_polygons = createObjectFootprints(predicted_objects, 0.0, 0.0);
  }
  return predicted_path_polygons;
}

linestring_t createMaxExpansionLine(const Path & path, const double max_expansion_distance)
{
  namespace strategy = boost::geometry::strategy::buffer;
  linestring_t max_expansion_line;
  if (max_expansion_distance > 0.0) {
    multipolygon_t polygons;
    linestring_t path_ls;
    for (const auto & p : path.points) path_ls.push_back({p.pose.position.x, p.pose.position.y});
    boost::geometry::buffer(
      path_ls, polygons, strategy::distance_symmetric<double>(max_expansion_distance),
      strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
      strategy::point_square());
    if (!polygons.empty()) {
      const auto & polygon = polygons.front();
      max_expansion_line.insert(
        max_expansion_line.end(), polygon.outer().begin(), polygon.outer().end());
    }
  }
  return max_expansion_line;
}
}  // namespace drivable_area_expander
