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
multipolygon_t filterFootprint(const multipolygon_t & footprint, const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines) {
  multipolygon_t filtered_footprints;
  // TODO(Maxime): crop rather than discard
  for(const auto & f : footprint)
    if(!boost::geometry::intersects(f, uncrossable_lines) && boost::geometry::disjoint(f, predicted_paths))
      filtered_footprints.push_back(f);
  return filtered_footprints;
}

multilinestring_t expandDrivableArea(std::vector<Point> & left_bound, std::vector<Point> & right_bound, const multipolygon_t & footprint)
{
  const auto to_point = [](const auto & p) { return point_t{p.x, p.y}; };
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(left_bound.size() + right_bound.size() + 1);
  for (const auto & p : left_bound) original_da_poly.outer().push_back(to_point(p));
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    original_da_poly.outer().push_back(to_point(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());
  // extend with filtered footprint
  multipolygon_t unions;
  auto extended_da_poly = original_da_poly;
  for(const auto & f : footprint) {
    unions.clear();
    boost::geometry::union_(extended_da_poly, f, unions);
    // assume that the footprints are not disjoint from the original drivable area
    extended_da_poly = unions.front();
  }
  // TODO(Maxime): extract left/right bound from da_poly
  left_bound.clear();
  right_bound.clear();
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
  multilinestring_t debug_ls;
  linestring_t ls;
  for (const auto & p : extended_da_poly.outer()) ls.push_back(p);
  debug_ls.push_back(ls);
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
