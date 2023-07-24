// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/expansion.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"
#include "interpolation/linear_interpolation.hpp"

#include <Eigen/Geometry>

#include <boost/geometry.hpp>

// for writing the svg file
#include <fstream>
#include <iostream>
// for the geometry types
#include <tier4_autoware_utils/geometry/geometry.hpp>
// for the svg mapper
#include <boost/geometry/io/svg/svg_mapper.hpp>
#include <boost/geometry/io/svg/write.hpp>

namespace drivable_area_expansion
{

void expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  const PredictedObjects & dynamic_objects, const route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & path_lanes)
{
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  multilinestring_t uncrossable_lines_in_range;
  const auto & p = path.points.front().point.pose.position;
  for (const auto & line : uncrossable_lines)
    if (boost::geometry::distance(line, point_t{p.x, p.y}) < params.max_path_arc_length)
      uncrossable_lines_in_range.push_back(line);
  const auto path_footprints = createPathFootprints(path, params);
  const auto predicted_paths = createObjectFootprints(dynamic_objects, params);
  const auto expansion_polygons =
    params.expansion_method == "lanelet"
      ? createExpansionLaneletPolygons(
          path_lanes, route_handler, path_footprints, predicted_paths, params)
      : createExpansionPolygons(
          path, path_footprints, predicted_paths, uncrossable_lines_in_range, params);
  const auto expanded_drivable_area = createExpandedDrivableAreaPolygon(path, expansion_polygons);
  updateDrivableAreaBounds(path, expanded_drivable_area);
}

point_t convert_point(const Point & p)
{
  return point_t{p.x, p.y};
}

Point convert_point(const point_t & p)
{
  return Point().set__x(p.x()).set__y(p.y());
}

polygon_t createExpandedDrivableAreaPolygon(
  const PathWithLaneId & path, const multipolygon_t & expansion_polygons)
{
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(path.left_bound.size() + path.right_bound.size() + 1);
  for (const auto & p : path.left_bound) original_da_poly.outer().push_back(convert_point(p));
  for (auto it = path.right_bound.rbegin(); it != path.right_bound.rend(); ++it)
    original_da_poly.outer().push_back(convert_point(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());

  multipolygon_t unions;
  auto expanded_da_poly = original_da_poly;
  for (const auto & p : expansion_polygons) {
    unions.clear();
    boost::geometry::union_(expanded_da_poly, p, unions);
    if (unions.size() == 1)  // union of overlapping polygons should produce a single polygon
      expanded_da_poly = unions[0];
  }
  return expanded_da_poly;
}

void copy_z_over_arc_length(
  const std::vector<geometry_msgs::msg::Point> & from, std::vector<geometry_msgs::msg::Point> & to)
{
  if (from.empty() || to.empty()) return;
  to.front().z = from.front().z;
  if (from.size() < 2 || to.size() < 2) return;
  to.back().z = from.back().z;
  auto i_from = 1lu;
  auto s_from = tier4_autoware_utils::calcDistance2d(from[0], from[1]);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (auto i_to = 1lu; i_to + 1 < to.size(); ++i_to) {
    s_to += tier4_autoware_utils::calcDistance2d(to[i_to - 1], to[i_to]);
    for (; s_from < s_to && i_from + 1 < from.size(); ++i_from) {
      s_from_prev = s_from;
      s_from += tier4_autoware_utils::calcDistance2d(from[i_from], from[i_from + 1]);
    }
    if (s_from - s_from_prev != 0.0) {
      const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
      to[i_to].z = interpolation::lerp(from[i_from - 1].z, from[i_from].z, ratio);
    } else {
      to[i_to].z = to[i_to - 1].z;
    }
  }
}

void updateDrivableAreaBounds(PathWithLaneId & path, const polygon_t & expanded_drivable_area)
{
  const auto original_left_bound = path.left_bound;
  const auto original_right_bound = path.right_bound;
  const auto is_left_of_segment = [](const point_t & a, const point_t & b, const point_t & p) {
    return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x()) > 0;
  };

  const auto start_segment =
    segment_t{convert_point(path.left_bound.front()), convert_point(path.right_bound.front())};
  const auto end_segment =
    segment_t{convert_point(path.left_bound.back()), convert_point(path.right_bound.back())};
  point_t start_segment_center;
  boost::geometry::centroid(start_segment, start_segment_center);
  const auto path_start_segment =
    segment_t{start_segment_center, convert_point(path.points[1].point.pose.position)};
  point_t end_segment_center;
  boost::geometry::centroid(end_segment, end_segment_center);
  const auto path_end_segment =
    segment_t{convert_point(path.points.back().point.pose.position), end_segment_center};
  const auto is_left_of_path_start = [&](const point_t & p) {
    return is_left_of_segment(
      convert_point(path.points[0].point.pose.position),
      convert_point(path.points[1].point.pose.position), p);
  };
  const auto is_left_of_path_end = [&](const point_t & p) {
    return is_left_of_segment(
      convert_point(path.points.back().point.pose.position), end_segment_center, p);
  };
  const auto segment_to_line_intersection =
    [](const auto p1, const auto p2, const auto q1, const auto q2) -> std::optional<point_t> {
    const auto line = Eigen::Hyperplane<double, 2>::Through(q1, q2);
    const auto segment = Eigen::Hyperplane<double, 2>::Through(p1, p2);
    const auto intersection = line.intersection(segment);
    std::optional<point_t> result;
    const auto is_on_segment =
      (p1.x() <= p2.x() ? intersection.x() >= p1.x() && intersection.x() <= p2.x()
                        : intersection.x() <= p1.x() && intersection.x() >= p2.x()) &&
      (p1.y() <= p2.y() ? intersection.y() >= p1.y() && intersection.y() <= p2.y()
                        : intersection.y() <= p1.y() && intersection.y() >= p2.y());
    if (is_on_segment) result = point_t{intersection.x(), intersection.y()};
    return result;
  };
  struct Intersection
  {
    ring_t::const_iterator segment_it;
    point_t intersection_point;
    double distance = std::numeric_limits<double>::max();
  };
  const auto & da = expanded_drivable_area.outer();
  Intersection start_left;
  start_left.segment_it = da.end();
  Intersection end_left;
  end_left.segment_it = da.end();
  Intersection start_right;
  start_right.segment_it = da.end();
  Intersection end_right;
  end_right.segment_it = da.end();

  // Declare a stream and an SVG mapper
  std::ofstream svg("/home/mclement/Pictures/debug.svg");
  boost::geometry::svg_mapper<tier4_autoware_utils::Point2d> mapper(svg, 400, 400);
  for (auto it = da.begin(); it != da.end(); ++it) {
    if (boost::geometry::distance(*it, start_segment.first) < 1e-3) {
      start_left.intersection_point = *it;
      start_left.segment_it = it;
      start_left.distance = 0.0;
    } else if (boost::geometry::distance(*it, start_segment.second) < 1e-3) {
      start_right.intersection_point = *it;
      start_right.segment_it = it;
      start_right.distance = 0.0;
    } else if (boost::geometry::distance(*it, end_segment.first) < 1e-3) {
      end_left.intersection_point = *it;
      end_left.segment_it = it;
      end_left.distance = 0.0;
    } else if (boost::geometry::distance(*it, end_segment.second) < 1e-3) {
      end_right.intersection_point = *it;
      end_right.segment_it = it;
      end_right.distance = 0.0;
    }
    const auto inter_start =
      std::next(it) == da.end()
        ? segment_to_line_intersection(*it, da.front(), start_segment.first, start_segment.second)
        : segment_to_line_intersection(
            *it, *std::next(it), start_segment.first, start_segment.second);
    if (inter_start) {
      const auto dist = boost::geometry::distance(*inter_start, path_start_segment);
      if (is_left_of_path_start(*inter_start)) {
        if (dist < start_left.distance) {
          start_left.intersection_point = *inter_start;
          start_left.segment_it = it;
          start_left.distance = dist;
        }
      } else {
        if (dist < start_right.distance) {
          start_right.intersection_point = *inter_start;
          start_right.segment_it = it;
          start_right.distance = dist;
        }
      }
    }
    const auto inter_end =
      std::next(it) == da.end()
        ? segment_to_line_intersection(*it, da.front(), end_segment.first, end_segment.second)
        : segment_to_line_intersection(*it, *std::next(it), end_segment.first, end_segment.second);
    if (inter_end) {
      const auto dist = boost::geometry::distance(*inter_end, path_end_segment);
      if (is_left_of_path_end(*inter_end)) {
        if (dist < end_left.distance) {
          end_left.intersection_point = *inter_end;
          end_left.segment_it = it;
          end_left.distance = dist;
        }
      } else {
        if (dist < end_right.distance) {
          end_right.intersection_point = *inter_end;
          end_right.segment_it = it;
          end_right.distance = dist;
        }
      }
    }
  }
  mapper.add(expanded_drivable_area);
  mapper.add(path_start_segment);
  mapper.add(path_end_segment);
  mapper.add(start_segment);
  mapper.add(end_segment);
  mapper.map(expanded_drivable_area, "fill-opacity:0.0;fill:black;stroke:black;stroke-width:1", 1);
  mapper.map(start_segment, "fill-opacity:0.5;fill:blue;stroke:blue;stroke-width:2", 2);
  mapper.map(end_segment, "fill-opacity:0.5;fill:blue;stroke:blue;stroke-width:2", 2);
  mapper.map(path_start_segment, "fill-opacity:0.5;fill:green;stroke:green;stroke-width:2", 2);
  mapper.map(path_end_segment, "fill-opacity:0.5;fill:green;stroke:green;stroke-width:2", 2);
  mapper.map(
    start_left.intersection_point, "fill-opacity:0.5;fill:red;stroke:red;stroke-width:2", 2);
  mapper.map(
    start_right.intersection_point, "fill-opacity:0.5;fill:red;stroke:red;stroke-width:2", 2);
  mapper.map(end_left.intersection_point, "fill-opacity:0.5;fill:red;stroke:red;stroke-width:2", 2);
  mapper.map(
    end_right.intersection_point, "fill-opacity:0.5;fill:red;stroke:red;stroke-width:2", 2);

  if (
    start_left.segment_it == da.end() || start_right.segment_it == da.end() ||
    end_left.segment_it == da.end() || end_right.segment_it == da.end()) {
    std::cerr << (start_left.segment_it == da.end()) << " " << (start_right.segment_it == da.end())
              << " " << (end_left.segment_it == da.end()) << " "
              << (end_right.segment_it == da.end()) << std::endl;
    return;
  }

  path.left_bound.clear();
  path.right_bound.clear();
  path.left_bound.push_back(convert_point(start_left.intersection_point));
  path.right_bound.push_back(convert_point(start_right.intersection_point));
  if (!boost::geometry::equals(start_right.intersection_point, *start_right.segment_it))
    path.right_bound.push_back(convert_point(*start_right.segment_it));
  if (start_left.segment_it < end_left.segment_it) {
    for (auto it = std::next(start_left.segment_it); it <= end_left.segment_it; ++it)
      path.left_bound.push_back(convert_point(*it));
  } else {
    for (auto it = std::next(start_left.segment_it); it < da.end(); ++it)
      path.left_bound.push_back(convert_point(*it));
    for (auto it = da.begin(); it <= end_left.segment_it; ++it)
      path.left_bound.push_back(convert_point(*it));
  }
  if (!boost::geometry::equals(end_left.intersection_point, *end_left.segment_it))
    path.left_bound.push_back(convert_point(end_left.intersection_point));
  if (start_right.segment_it < end_right.segment_it) {
    for (auto it = std::prev(start_right.segment_it); it >= da.begin(); --it)
      path.right_bound.push_back(convert_point(*it));
    for (auto it = std::prev(da.end()); it > end_right.segment_it; --it)
      path.right_bound.push_back(convert_point(*it));
  } else {
    for (auto it = std::prev(start_right.segment_it); it > end_right.segment_it; --it)
      path.right_bound.push_back(convert_point(*it));
  }
  if (!boost::geometry::equals(end_right.intersection_point, *std::next(end_right.segment_it)))
    path.right_bound.push_back(convert_point(end_right.intersection_point));

  // remove possible duplicated points
  const auto point_cmp = [](const auto & p1, const auto & p2) {
    return p1.x == p2.x && p1.y == p2.y;
  };
  std::unique(path.left_bound.begin(), path.left_bound.end(), point_cmp);
  std::unique(path.right_bound.begin(), path.right_bound.end(), point_cmp);
  copy_z_over_arc_length(original_left_bound, path.left_bound);
  copy_z_over_arc_length(original_right_bound, path.right_bound);

  for (const auto & p : path.right_bound) {
    mapper.add(convert_point(p));
    mapper.map(convert_point(p), "fill-opacity:0.5;fill:grey;stroke:grey;stroke-width:2", 1);
  }
}

}  // namespace drivable_area_expansion
