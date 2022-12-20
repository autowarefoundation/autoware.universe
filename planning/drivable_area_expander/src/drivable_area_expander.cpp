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
multilinestring_t expandDrivableArea(
  std::vector<Point> & left_bound, std::vector<Point> & right_bound, const polygon_t & footprint,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const point_t & origin)
{
  const auto to_point = [](const auto & p) { return point_t{p.x, p.y}; };
  polygon_t poly;
  poly.outer().reserve(left_bound.size() + right_bound.size() + 1);
  for (const auto & p : left_bound) poly.outer().push_back(to_point(p));
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    poly.outer().push_back(to_point(*it));
  poly.outer().push_back(poly.outer().front());
  // cut uncrossable lines
  // TODO(Maxime)
  // extend with footprint
  multilinestring_t debug_ls;
  multipolygon_t drivable_area_poly;
  boost::geometry::union_(poly, footprint, drivable_area_poly);
  // remove predicted_paths
  multipolygon_t diffs;
  boost::geometry::difference(drivable_area_poly.front(), predicted_paths, diffs);
  for (const auto & diff : diffs) {
    if (boost::geometry::within(origin, diff)) drivable_area_poly = {diff};
  }
  // TODO(Maxime): extract left/right bound from drivable_area_poly
  // left_bound.clear();
  // right_bound.clear();
  // TODO(Maxime): what about the z value ?
  Point point;
  /*
  for(const auto & p : lb) {
    point.x = p.x();
    point.y = p.y();
    left_bound.push_back(point);
  }
  for(const auto & p : rb) {
    point.x = p.x();
    point.y = p.y();
    right_bound.push_back(point);
  }
  */
  for (const auto & poly : drivable_area_poly) {
    linestring_t ls;
    for (const auto & p : poly.outer()) ls.push_back(p);
    debug_ls.push_back(ls);
  }
  return debug_ls;
}

multipolygon_t createPredictedPathPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & predicted_objects,
  const ExpansionParameters & params)
{
  multipolygon_t predicted_path_polygons;
  if (params.avoid_dynamic_objects) {
    predicted_path_polygons = createObjectFootprints(predicted_objects, params);
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
