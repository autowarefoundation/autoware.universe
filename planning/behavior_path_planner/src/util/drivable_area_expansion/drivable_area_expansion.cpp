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

#include "behavior_path_planner/util/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace drivable_area_expansion
{
bool expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  multilinestring_t uncrossable_lines,
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_objects)
{
  auto path_footprints = createPathFootprints(path, params);
  uncrossable_lines.push_back(createMaxExpansionLine(path, params.max_expansion_distance));
  const auto predicted_paths = createObjectFootprints(dynamic_objects, params);
  const auto filtered_footprint = filterFootprint(
    path_footprints, predicted_paths, uncrossable_lines, params.avoid_linestring_dist);

  return expandDrivableArea(path.left_bound, path.right_bound, filtered_footprint);
}

multipolygon_t filterFootprint(
  const std::vector<Footprint> & footprints, const std::vector<Footprint> & predicted_paths,
  const multilinestring_t & uncrossable_lines, const double dist_from_uncrossable_lines)
{
  namespace strategy = boost::geometry::strategy::buffer;
  multipolygon_t filtered_footprints;
  polygon_t filtered_footprint;
  multipolygon_t cuts;
  multipolygon_t uncrossable_polys;
  boost::geometry::buffer(
    uncrossable_lines, uncrossable_polys,
    strategy::distance_symmetric<double>(std::max(dist_from_uncrossable_lines, 0.01)),
    strategy::side_straight(), strategy::join_miter(), strategy::end_flat(),
    strategy::point_square());
  for (const auto & f : footprints) {
    filtered_footprint = f.footprint;
    cuts.clear();
    boost::geometry::difference(filtered_footprint, uncrossable_polys, cuts);
    for (const auto & cut : cuts) {
      if (boost::geometry::within(f.origin, cut)) {
        filtered_footprint = cut;
        break;
      }
    }
    for (const auto & p : predicted_paths) {
      cuts.clear();
      boost::geometry::difference(filtered_footprint, p.footprint, cuts);
      for (const auto & cut : cuts) {
        if (boost::geometry::within(f.origin, cut)) {
          filtered_footprint = cut;
          break;
        }
      }
    }
    if (!filtered_footprint.outer().empty()) filtered_footprints.push_back(filtered_footprint);
  }
  return filtered_footprints;
}

bool expandDrivableArea(
  std::vector<Point> & left_bound, std::vector<Point> & right_bound,
  const multipolygon_t & footprint)
{
  const auto to_point_t = [](const auto & p) { return point_t{p.x, p.y}; };
  const auto to_point = [](const auto & p) { return Point().set__x(p.x()).set__y(p.y()); };
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(left_bound.size() + right_bound.size() + 1);
  for (const auto & p : left_bound) original_da_poly.outer().push_back(to_point_t(p));
  for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it)
    original_da_poly.outer().push_back(to_point_t(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());
  // extend with filtered footprint
  multipolygon_t unions;
  auto extended_da_poly = original_da_poly;
  for (const auto & f : footprint) {
    unions.clear();
    boost::geometry::union_(extended_da_poly, f, unions);
    // algorithm only works if the footprints are not disjoint from the original drivable area
    if (unions.size() != 1) {
      return false;
    } else {
      extended_da_poly = unions.front();
    }
  }
  boost::geometry::correct(extended_da_poly);
  // remove the duplicated point (front == back) to prevent issue when splitting into left/right
  extended_da_poly.outer().resize(extended_da_poly.outer().size() - 1);
  // extract left and right bounds: find the points closest to the original start and end points
  const auto begin = extended_da_poly.outer().begin();
  const auto end = extended_da_poly.outer().end();
  auto lf = end;
  double lf_min_dist = std::numeric_limits<double>::max();
  auto rf = end;
  double rf_min_dist = std::numeric_limits<double>::max();
  auto lb = end;
  double lb_min_dist = std::numeric_limits<double>::max();
  auto rb = end;
  double rb_min_dist = std::numeric_limits<double>::max();
  for (auto it = extended_da_poly.outer().begin(); it != extended_da_poly.outer().end(); ++it) {
    const auto lf_dist = boost::geometry::distance(to_point_t(left_bound.front()), *it);
    const auto rf_dist = boost::geometry::distance(to_point_t(right_bound.front()), *it);
    const auto lb_dist = boost::geometry::distance(to_point_t(left_bound.back()), *it);
    const auto rb_dist = boost::geometry::distance(to_point_t(right_bound.back()), *it);
    if (lf_dist < lf_min_dist) {
      lf = it;
      lf_min_dist = lf_dist;
    }
    if (rf_dist < rf_min_dist) {
      rf = it;
      rf_min_dist = rf_dist;
    }
    if (lb_dist < lb_min_dist) {
      lb = it;
      lb_min_dist = lb_dist;
    }
    if (rb_dist < rb_min_dist) {
      rb = it;
      rb_min_dist = rb_dist;
    }
  }
  std::vector<Point> extended_left_bound;
  std::vector<Point> extended_right_bound;
  // NOTE: clockwise ordering -> positive increment for left bound, negative for right bound
  if (lf < lb) {
    for (auto it = lf; it <= lb; ++it) extended_left_bound.push_back(to_point(*it));
  } else {  // loop back
    for (auto it = lf; it != end; ++it) extended_left_bound.push_back(to_point(*it));
    for (auto it = begin; it <= lb; ++it) extended_left_bound.push_back(to_point(*it));
  }
  if (rf > rb) {
    for (auto it = rf; it >= rb; --it) extended_right_bound.push_back(to_point(*it));
  } else {  // loop back
    for (auto it = rf; it >= begin; --it) extended_right_bound.push_back(to_point(*it));
    for (auto it = end - 1; it >= rb; --it) extended_right_bound.push_back(to_point(*it));
  }
  left_bound = extended_left_bound;
  right_bound = extended_right_bound;
  return true;
}

std::vector<Footprint> createPathFootprints(
  const PathWithLaneId & path, const DrivableAreaExpansionParameters & params)
{
  const auto left = params.ego_left_offset + params.ego_extra_left_offset;
  const auto right = params.ego_right_offset - params.ego_extra_right_offset;
  const auto rear = params.ego_rear_offset - params.ego_extra_rear_offset;
  const auto front = params.ego_front_offset + params.ego_extra_front_offset;
  polygon_t base_footprint;
  base_footprint.outer() = {
    point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left},
    point_t{front, left}};
  std::vector<Footprint> footprints;
  footprints.reserve(path.points.size());
  for (const auto & point : path.points)
    footprints.push_back(createFootprint(point.point.pose, base_footprint));
  return footprints;
}

linestring_t createMaxExpansionLine(
  const PathWithLaneId & path, const double max_expansion_distance)
{
  namespace strategy = boost::geometry::strategy::buffer;
  linestring_t max_expansion_line;
  if (max_expansion_distance > 0.0) {
    multipolygon_t polygons;
    linestring_t path_ls;
    for (const auto & p : path.points)
      path_ls.push_back({p.point.pose.position.x, p.point.pose.position.y});
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
}  // namespace drivable_area_expansion
