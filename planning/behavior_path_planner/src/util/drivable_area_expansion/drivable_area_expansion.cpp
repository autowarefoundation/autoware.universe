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
#include "behavior_path_planner/util/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/path_projection.hpp"
#include "behavior_path_planner/util/drivable_area_expansion/types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <fstream>
#include <iostream>

namespace drivable_area_expansion
{

double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multilinestring_t & limit_lines)
{
  auto dist_limit = std::numeric_limits<double>::max();
  for (const auto & line : limit_lines) {
    for (const auto & p : line) {
      if (boost::geometry::within(p, expansion_polygon)) {
        dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
      }
    }
  }
  multipoint_t intersections;
  boost::geometry::intersection(expansion_polygon, limit_lines, intersections);
  for (const auto & p : intersections) {
    dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
  }
  return dist_limit;
}

double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multipolygon_t & limit_polygons)
{
  auto dist_limit = std::numeric_limits<double>::max();
  for (const auto & polygon : limit_polygons) {
    for (const auto & p : polygon.outer()) {
      if (boost::geometry::within(p, expansion_polygon)) {
        dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
      }
    }
  }
  for (const auto & polygon : limit_polygons) {
    multipoint_t intersections;
    boost::geometry::intersection(expansion_polygon, polygon, intersections);
    for (const auto & p : intersections) {
      dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
    }
  }
  return dist_limit;
}

polygon_t createExpansionPolygon(
  const linestring_t & base_ls, const double dist, const bool is_left_side)
{
  namespace strategy = boost::geometry::strategy::buffer;
  multipolygon_t polygons;
  // when setting 0.0 on one side the bg::buffer function may not return anything
  constexpr auto zero = 0.1;
  const auto left_dist = is_left_side ? dist : zero;
  const auto right_dist = !is_left_side ? dist : zero;
  const auto distance_strategy = strategy::distance_asymmetric<double>(left_dist, right_dist);
  boost::geometry::buffer(
    base_ls, polygons, distance_strategy, strategy::side_straight(), strategy::join_miter(),
    strategy::end_flat(), strategy::point_square());
  return polygons.front();
}

multipolygon_t createExpansionLaneletPolygons(
  const PathWithLaneId & path, const lanelet::ConstLanelets & path_lanes,
  const route_handler::RouteHandler & route_handler, const multipolygon_t & path_footprints,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const DrivableAreaExpansionParameters & params)
{
  multipolygon_t expansion_polygons;
  lanelet::ConstLanelets candidates;
  const auto already_added = [&](const auto & ll) {
    return std::find_if(candidates.begin(), candidates.end(), [&](const auto & l) {
             return ll.id() == l.id();
           }) != candidates.end();
  };
  const auto add_if_valid = [&](const auto & ll, const auto is_left) {
    const auto bound_to_check = is_left ? ll.rightBound() : ll.leftBound();
    if (!already_added(ll) && !hasTypes(bound_to_check, params.avoid_linestring_types))
      candidates.push_back(ll);
  };
  std::cout << path_lanes.size() << std::endl;
  for (const auto & current_ll : path_lanes) {
    const auto left_ll = route_handler.getLeftLanelet(current_ll);
    if (left_ll) add_if_valid(left_ll.get(), true);
    for (const auto & ll : route_handler.getRoutingGraphPtr()->lefts(current_ll))
      add_if_valid(ll, true);
    const auto right_ll = route_handler.getRightLanelet(current_ll);
    if (right_ll) add_if_valid(right_ll.get(), false);
    for (const auto & ll : route_handler.getRoutingGraphPtr()->rights(current_ll))
      add_if_valid(ll, false);
  }
  std::cout << candidates.size() << std::endl;
  for (const auto & footprint : path_footprints) {
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
      polygon_t poly;
      for (const auto & p : it->polygon2d()) poly.outer().emplace_back(p.x(), p.y());
      boost::geometry::correct(poly);
      if (boost::geometry::intersects(footprint, poly)) {
        expansion_polygons.push_back(poly);
        candidates.erase(it);
        --it;
      }
    }
  }
  return expansion_polygons;
}

multipolygon_t createExpansionPolygons(
  const PathWithLaneId & path, const multipolygon_t & path_footprints,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const DrivableAreaExpansionParameters & params)
{
  linestring_t path_ls;
  linestring_t left_ls;
  linestring_t right_ls;
  for (const auto & p : path.points)
    path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
  for (const auto & p : path.left_bound) left_ls.emplace_back(p.x, p.y);
  for (const auto & p : path.right_bound) right_ls.emplace_back(p.x, p.y);
  const auto path_length = static_cast<double>(boost::geometry::length(path_ls));

  // TODO(Maxime): make it a function
  const auto calculate_arc_length_range_and_distance = [&](
                                                         const auto & footprint, const auto & bound,
                                                         const bool is_left) {
    multipoint_t intersections;
    double expansion_dist = 0.0;
    double from_arc_length = std::numeric_limits<double>::max();
    double to_arc_length = std::numeric_limits<double>::min();
    boost::geometry::intersection(footprint, bound, intersections);
    if (!intersections.empty()) {
      for (const auto & intersection : intersections) {
        const auto projection = point_to_linestring_projection(intersection, path_ls);
        // TODO(Maxime): tmp fix for points before/after the path
        if (projection.arc_length <= 0.0 || projection.arc_length >= path_length) continue;
        from_arc_length = std::min(from_arc_length, projection.arc_length);
        to_arc_length = std::max(to_arc_length, projection.arc_length);
      }
      for (const auto & p : footprint.outer()) {
        const auto projection = point_to_linestring_projection(p, path_ls);
        // TODO(Maxime): tmp fix for points before/after the path
        if (projection.arc_length <= 0.0 || projection.arc_length >= path_length) continue;
        // we make sure the footprint point is on the correct side
        if (is_left == projection.distance > 0 && std::abs(projection.distance) > expansion_dist) {
          expansion_dist = std::abs(projection.distance);
          from_arc_length = std::min(from_arc_length, projection.arc_length);
          to_arc_length = std::max(to_arc_length, projection.arc_length);
        }
      }
    }
    return std::array<double, 3>({from_arc_length, to_arc_length, expansion_dist});
  };

  multipolygon_t expansion_polygons;
  for (const auto & footprint : path_footprints) {
    bool is_left = true;
    for (const auto & bound : {left_ls, right_ls}) {
      auto [from_arc_length, to_arc_length, footprint_dist] =
        calculate_arc_length_range_and_distance(footprint, bound, is_left);
      if (footprint_dist > 0.0) {
        from_arc_length -= params.extra_arc_length;
        to_arc_length += params.extra_arc_length;
        from_arc_length = std::max(0.0, from_arc_length);
        to_arc_length = std::min(path_length, to_arc_length);
        const auto base_ls = sub_linestring(path_ls, from_arc_length, to_arc_length);
        const auto expansion_dist = params.max_expansion_distance != 0.0
                                      ? std::min(params.max_expansion_distance, footprint_dist)
                                      : footprint_dist;
        auto expansion_polygon = createExpansionPolygon(base_ls, expansion_dist, is_left);
        auto limited_dist = expansion_dist;
        const auto uncrossable_dist_limit =
          calculateDistanceLimit(base_ls, expansion_polygon, uncrossable_lines);
        if (uncrossable_dist_limit < limited_dist) {
          limited_dist = uncrossable_dist_limit;
          if (params.compensate_uncrossable_lines) {
            const auto compensation_dist =
              footprint_dist - limited_dist + params.compensate_extra_dist;
            polygon_t compensation_polygon =
              createExpansionPolygon(base_ls, compensation_dist, !is_left);
            double dist_limit = std::min(
              compensation_dist,
              calculateDistanceLimit(base_ls, compensation_polygon, uncrossable_lines));
            if (params.avoid_dynamic_objects)
              dist_limit = std::min(
                dist_limit, calculateDistanceLimit(base_ls, compensation_polygon, predicted_paths));
            if (dist_limit < compensation_dist)
              compensation_polygon = createExpansionPolygon(base_ls, dist_limit, !is_left);
            expansion_polygons.push_back(compensation_polygon);
          }
        }
        if (params.avoid_dynamic_objects)
          limited_dist = std::min(
            limited_dist, calculateDistanceLimit(base_ls, expansion_polygon, predicted_paths));
        if (limited_dist < expansion_dist)
          expansion_polygon = createExpansionPolygon(base_ls, limited_dist, is_left);
        expansion_polygons.push_back(expansion_polygon);
      }
      is_left = false;
    }
  }
  return expansion_polygons;
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

bool updateDrivableAreaBounds(PathWithLaneId & path, const polygon_t & expanded_drivable_area)
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
  return true;
}

multipolygon_t createPathFootprints(
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
  multipolygon_t footprints;
  // skip the last footprint as its orientation is usually wrong
  footprints.reserve(path.points.size() - 1);
  for (auto it = path.points.begin(); std::next(it) != path.points.end(); ++it)
    footprints.push_back(createFootprint(it->point.pose, base_footprint));
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
      path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
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

bool expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_objects,
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelets & path_lanes)
{
  const auto start = std::chrono::high_resolution_clock::now();
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  const auto path_footprints = createPathFootprints(path, params);
  const auto predicted_paths = createObjectFootprints(dynamic_objects, params);
  const auto expansion_polygons =
    params.expansion_method == "lanelet"
      ? createExpansionLaneletPolygons(
          path, path_lanes, route_handler, path_footprints, predicted_paths, uncrossable_lines,
          params)
      : createExpansionPolygons(path, path_footprints, predicted_paths, uncrossable_lines, params);
  const auto expanded_drivable_area = createExpandedDrivableAreaPolygon(path, expansion_polygons);
  const auto debug_svg = false;
  if (debug_svg) {
    // Declare a stream and an SVG mapper
    std::ofstream svg("/home/mclement/Pictures/debug.svg");
    boost::geometry::svg_mapper<point_t> mapper(svg, 400, 400);

    for (const auto & p : path_footprints) mapper.add(p);
    for (const auto & p : expansion_polygons) mapper.add(p);
    for (const auto & l : uncrossable_lines) mapper.add(l);
    // blue path footprints
    for (const auto & p : path_footprints)
      mapper.map(p, "fill-opacity:0.1;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    // red expansion polygons
    for (const auto & p : expansion_polygons)
      mapper.map(p, "fill-opacity:0.2;stroke-opacity:0.2;fill:red;stroke:red;stroke-width:1", 1);
    // black uncrossable line
    for (const auto & l : uncrossable_lines)
      mapper.map(
        l, "fill-opacity:0.2;stroke-opacity:0.2;fill:black;stroke:black;stroke-width:2", 1);
  }
  if (debug_svg) {
    std::ofstream svg_inputs("/home/mclement/Pictures/inputs.svg");
    boost::geometry::svg_mapper<point_t> mapper(svg_inputs, 400, 400);
    linestring_t left;
    linestring_t right;
    for (const auto & p : path.left_bound) left.emplace_back(p.x, p.y);
    for (const auto & p : path.right_bound) right.emplace_back(p.x, p.y);
    mapper.add(left);
    mapper.add(right);
    for (const auto & p : path.points)
      mapper.add(point_t(p.point.pose.position.x, p.point.pose.position.y));
    for (const auto & l : uncrossable_lines) mapper.add(l);
    mapper.map(left, "fill-opacity:0.2;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    mapper.map(
      right, "fill-opacity:0.2;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    for (const auto & p : path.points)
      mapper.map(
        point_t(p.point.pose.position.x, p.point.pose.position.y),
        "fill-opacity:0.2;stroke-opacity:1.0;fill:black;stroke:black;stroke-width:1", 1);
    for (const auto & l : uncrossable_lines)
      mapper.map(l, "fill-opacity:0.2;stroke-opacity:0.5;fill:grey;stroke:black;stroke-width:2", 1);
    if (!path_footprints.empty()) {
      mapper.add(path_footprints.front());
      mapper.map(
        path_footprints.front(),
        "fill-opacity:0.1;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    }
  }

  const auto return_bool = updateDrivableAreaBounds(path, expanded_drivable_area);

  if (debug_svg) {
    std::ofstream svg_outputs("/home/mclement/Pictures/outputs.svg");
    boost::geometry::svg_mapper<point_t> mapper(svg_outputs, 400, 400);
    linestring_t left;
    linestring_t right;
    for (const auto & p : path.left_bound) left.emplace_back(p.x, p.y);
    for (const auto & p : path.right_bound) right.emplace_back(p.x, p.y);
    for (const auto & l : uncrossable_lines) mapper.add(l);
    mapper.add(left);
    mapper.add(right);
    for (const auto & p : path.points)
      mapper.add(point_t(p.point.pose.position.x, p.point.pose.position.y));
    mapper.map(left, "fill-opacity:0.2;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    mapper.map(
      right, "fill-opacity:0.2;stroke-opacity:0.5;fill:blue;stroke:blue;stroke-width:1", 1);
    for (const auto & p : path.points)
      mapper.map(
        point_t(p.point.pose.position.x, p.point.pose.position.y),
        "fill-opacity:0.2;stroke-opacity:1.0;fill:black;stroke:black;stroke-width:1", 1);
    // for (const auto & l : uncrossable_lines)
    //   mapper.map(
    //     l, "fill-opacity:0.2;stroke-opacity:0.5;fill:grey;stroke:black;stroke-width:2", 1);
  }
  const auto end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us\n";
  return return_bool;
}
}  // namespace drivable_area_expansion
