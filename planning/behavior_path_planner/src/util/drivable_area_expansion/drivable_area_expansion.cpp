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

#include "behavior_path_planner/util/drivable_area_expansion/drivable_area_expansion.hpp"

#include "behavior_path_planner/util/drivable_area_expansion/expansion.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace drivable_area_expansion
{

void expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  const PredictedObjects & dynamic_objects, const route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & path_lanes)
{
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  const auto path_footprints = createPathFootprints(path, params);
  const auto predicted_paths = createObjectFootprints(dynamic_objects, params);
  const auto expansion_polygons =
    params.expansion_method == "lanelet"
      ? createExpansionLaneletPolygons(
          path_lanes, route_handler, path_footprints, predicted_paths, params)
      : createExpansionPolygons(path, path_footprints, predicted_paths, uncrossable_lines, params);
  const auto expanded_drivable_area = createExpandedDrivableAreaPolygon(path, expansion_polygons);
  updateDrivableAreaBounds(path, expanded_drivable_area);
}

polygon_t createExpandedDrivableAreaPolygon(
  const PathWithLaneId & path, const multipolygon_t & expansion_polygons)
{
  const auto to_point_t = [](const auto & p) { return point_t{p.x, p.y}; };
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(path.left_bound.size() + path.right_bound.size() + 1);
  for (const auto & p : path.left_bound) original_da_poly.outer().push_back(to_point_t(p));
  for (auto it = path.right_bound.rbegin(); it != path.right_bound.rend(); ++it)
    original_da_poly.outer().push_back(to_point_t(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());

  multipolygon_t unions;
  auto expanded_da_poly = original_da_poly;
  for (const auto & p : expansion_polygons) {
    unions.clear();
    boost::geometry::union_(expanded_da_poly, p, unions);
    if (unions.size() != 1)  // union of overlapping polygons should produce a single polygon
      continue;
    else
      expanded_da_poly = unions[0];
  }
  return expanded_da_poly;
}

void updateDrivableAreaBounds(PathWithLaneId & path, const polygon_t & expanded_drivable_area)
{
  const auto to_point_t = [](const auto & p) { return point_t{p.x, p.y}; };
  const auto to_point = [](const auto & p) { return Point().set__x(p.x()).set__y(p.y()); };
  // extract left and right bounds: find the transitions between left and right side closest to the
  // start and end of the path
  const auto is_left_of_segment = [](const point_t & a, const point_t & b, const point_t & p) {
    return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x()) > 0;
  };
  const auto is_left_of_path_start = [&](const point_t & p) {
    return is_left_of_segment(
      to_point_t(path.points[0].point.pose.position),
      to_point_t(path.points[1].point.pose.position), p);
  };
  const auto is_left_of_path_end = [&, size = path.points.size()](const point_t & p) {
    return is_left_of_segment(
      to_point_t(path.points[size - 2].point.pose.position),
      to_point_t(path.points[size - 1].point.pose.position), p);
  };
  const auto dist_to_path_start = [start = to_point_t(path.points.front().point.pose.position)](
                                    const auto & p) { return boost::geometry::distance(start, p); };
  const auto dist_to_path_end = [end = to_point_t(path.points.back().point.pose.position)](
                                  const auto & p) { return boost::geometry::distance(end, p); };
  const auto begin = expanded_drivable_area.outer().begin();
  const auto end = expanded_drivable_area.outer().end();
  double min_start_dist = std::numeric_limits<double>::max();
  auto start_it = end;
  double min_end_dist = std::numeric_limits<double>::max();
  auto end_it = end;
  for (auto it = begin; std::next(it) != end; ++it) {
    if (is_left_of_path_start(*it) != is_left_of_path_start(*std::next(it))) {
      const auto dist = dist_to_path_start(*it);
      if (dist < min_start_dist) {
        start_it = it;
        min_start_dist = dist;
      }
    }
    if (is_left_of_path_end(*it) != is_left_of_path_end(*std::next(it))) {
      const auto dist = dist_to_path_end(*it);
      if (dist < min_end_dist) {
        end_it = it;
        min_end_dist = dist;
      }
    }
  }
  std::vector<Point> expanded_left_bound;
  std::vector<Point> expanded_right_bound;
  const auto left_start = is_left_of_path_start(*start_it) ? start_it : std::next(start_it);
  const auto right_start = is_left_of_path_start(*start_it) ? std::next(start_it) : start_it;
  const auto left_end = is_left_of_path_end(*end_it) ? end_it : std::next(end_it);
  const auto right_end = is_left_of_path_end(*end_it) ? std::next(end_it) : end_it;
  // NOTE: clockwise ordering -> positive increment for left bound, negative for right bound
  if (left_start < left_end) {
    expanded_left_bound.reserve(std::distance(left_start, left_end));
    for (auto it = left_start; it <= left_end; ++it) expanded_left_bound.push_back(to_point(*it));
  } else {  // loop back
    expanded_left_bound.reserve(std::distance(left_start, end) + std::distance(begin, left_end));
    for (auto it = left_start; it != end; ++it) expanded_left_bound.push_back(to_point(*it));
    for (auto it = begin; it <= left_end; ++it) expanded_left_bound.push_back(to_point(*it));
  }
  if (right_start > right_end) {
    expanded_right_bound.reserve(std::distance(right_end, right_start));
    for (auto it = right_start; it >= right_end; --it)
      expanded_right_bound.push_back(to_point(*it));
  } else {  // loop back
    expanded_right_bound.reserve(std::distance(begin, right_start) + std::distance(right_end, end));
    for (auto it = right_start; it >= begin; --it) expanded_right_bound.push_back(to_point(*it));
    for (auto it = end - 1; it >= right_end; --it) expanded_right_bound.push_back(to_point(*it));
  }
  path.left_bound = expanded_left_bound;
  path.right_bound = expanded_right_bound;
}

}  // namespace drivable_area_expansion
