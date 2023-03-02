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

#ifndef SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
#define SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry/io/svg/write_svg.hpp>
#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <list>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using lanelet::ArcCoordinates;
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
using lanelet::BasicPolygon2d;
using lanelet::ConstLineString2d;
using lanelet::LaneletMapPtr;
using lanelet::geometry::fromArcCoordinates;
using lanelet::geometry::toArcCoordinates;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

namespace out_of_lane_utils
{
struct PlannerParam
{
  double overlap_min_dist;  // [m] minimum distance inside a lane for a footprint to be considered
                            // overlapping
  double dist_thr;          // [m]

  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance
};

struct Slowdown
{
  size_t target_path_idx;
  double velocity;
};

struct Overlap
{
  size_t path_idx;
  lanelet::Ids overlapped_lanes;
  // for each overlapped lane
  std::vector<double> overlap_distances;    // the distance inside the lane
  std::vector<double> overlap_arc_lengths;  // the min arc length along the lane
  std::vector<lanelet::BasicPolygons2d> overlapping_areas;
};

struct Interval
{
  size_t entering_path_idx;
  size_t exiting_path_idx;
  lanelet::Id lane_id;
};

typedef std::vector<Overlap> Overlaps;
typedef std::vector<Interval> Intervals;

struct DebugData
{
  std::vector<lanelet::BasicPolygon2d> footprints;
  Overlaps overlaps;
  Intervals intervals;
  std::vector<Pose> slowdown_poses;
  void resetData()
  {
    footprints.clear();
    overlaps.clear();
    intervals.clear();
    slowdown_poses.clear();
  }
};

/// @brief calculate the path footprints
/// @details the resulting polygon follows the format used by the lanelet library: clockwise order
/// and implicit closing edge
inline std::vector<BasicPolygon2d> calculate_path_footprints(
  const PathWithLaneId & path, const size_t first_idx, const PlannerParam & params)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  base_footprint.outer() = {
    {params.front_offset + params.extra_front_offset,
     params.left_offset + params.extra_left_offset},
    {params.front_offset + params.extra_front_offset,
     params.right_offset - params.extra_right_offset},
    {params.rear_offset - params.extra_rear_offset,
     params.right_offset - params.extra_right_offset},
    {params.rear_offset - params.extra_rear_offset, params.left_offset + params.extra_left_offset}};

  std::vector<BasicPolygon2d> path_footprints;
  path_footprints.reserve(path.points.size());
  for (auto i = first_idx; i < path.points.size(); ++i) {
    const auto & path_pose = path.points[i].point.pose;
    const auto angle = tf2::getYaw(path_pose.orientation);
    const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
    BasicPolygon2d footprint;
    for (const auto & p : rotated_footprint.outer())
      footprint.emplace_back(p.x() + path_pose.position.x, p.y() + path_pose.position.y);
    path_footprints.push_back(footprint);
  }
  return path_footprints;
}

inline Overlaps calculate_overlaps(
  const std::vector<BasicPolygon2d> & path_footprints, const lanelet::ConstLanelets & path_lanelets,
  const size_t idx_offset, const lanelet::ConstLanelets & lanelets, const PlannerParam & params)
{
  (void)params;
  Overlaps overlaps;
  lanelet::BasicPolygon2d path_lanelets_polygon;
  lanelet::BasicPolygons2d union_polygons;
  for (const auto & path_lanelet : path_lanelets) {
    union_polygons.clear();
    boost::geometry::union_(
      path_lanelets_polygon, path_lanelet.polygon2d().basicPolygon(), union_polygons);
    path_lanelets_polygon = union_polygons.front();
  }
  std::ofstream svg("/tmp/debug.svg");
  boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 800, 800);
  mapper.add(path_lanelets_polygon);
  for (auto i = 0UL; i < path_footprints.size(); ++i) {
    auto path_footprint = path_footprints[i];
    mapper.add(path_footprint);
    mapper.map(path_footprint, "opacity:0.5;fill-opacity:0.0;fill:red;stroke:red;stroke-width:1");
    lanelet::BasicPolygons2d outside_polys;
    boost::geometry::difference(path_footprint, path_lanelets_polygon, outside_polys);
    Overlap overlap;
    lanelet::BasicPolygons2d overlapping_areas;
    overlap.path_idx = idx_offset + i;
    for (const auto & lanelet : lanelets) {
      const auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
      auto inside_dist = 0.0;
      auto arc_length_along_lanelet = std::numeric_limits<double>::infinity();
      for (const auto & outside_poly : outside_polys) {
        mapper.add(outside_poly);
        mapper.map(outside_poly, "opacity:0.5;fill-opacity:0.2;fill:red;stroke:red;stroke-width:1");
        bool added = false;
        for (const auto & outside_point : outside_poly) {
          if (boost::geometry::within(outside_point, lanelet_polygon)) {
            if (!added) {  // TODO(Maxime): only for debug. to remove
              overlapping_areas.push_back(outside_poly);
              added = true;
            }
            // TODO(Maxime): inside distance is not correct
            inside_dist =
              std::min(inside_dist, boost::geometry::distance(outside_point, lanelet_polygon));
            arc_length_along_lanelet = std::min(
              arc_length_along_lanelet,
              lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), outside_point).length);
          }
        }
      }
      if (arc_length_along_lanelet < std::numeric_limits<double>::infinity()) {
        overlap.overlapped_lanes.push_back(lanelet.id());
        overlap.overlap_distances.push_back(inside_dist);
        overlap.overlap_arc_lengths.push_back(arc_length_along_lanelet);
        overlap.overlapping_areas.push_back(overlapping_areas);
      }
    }
    if (!overlap.overlapped_lanes.empty()) overlaps.push_back(overlap);
  }
  mapper.map(
    path_lanelets_polygon, "opacity:0.5;fill-opacity:0.2;fill:green;stroke:green;stroke-width:1");
  return overlaps;
}

/// @brief calculate overlapping intervals
/// @param overlaps overlaps sorted by increasing path index
/// @return overlapping intervals
inline Intervals calculate_overlapping_intervals(const Overlaps & overlaps)
{
  struct OpenedInterval
  {
    lanelet::Id lane_id;
    size_t index;
    bool should_keep_open;
  };
  std::list<OpenedInterval> opened_intervals;
  Intervals intervals;
  for (auto overlap_it = overlaps.cbegin(); overlap_it != overlaps.cend(); ++overlap_it) {
    // identify ids that need to be opened/closed/kept open
    for (auto & opened_interval : opened_intervals) opened_interval.should_keep_open = false;
    for (const auto & id : overlap_it->overlapped_lanes) {
      const auto opened_interval_it = std::find_if(
        opened_intervals.begin(), opened_intervals.end(),
        [&](const auto & i) { return i.lane_id == id; });
      if (opened_interval_it != opened_intervals.end()) {
        opened_interval_it->should_keep_open = true;
      } else {
        OpenedInterval opened_interval;
        opened_interval.lane_id = id;
        opened_interval.index = overlap_it->path_idx;
        opened_interval.should_keep_open = true;
        opened_intervals.push_back(opened_interval);
      }
    }
    // last overlap or gap with the next overlap: close all the opened intervals
    if (
      std::next(overlap_it) == overlaps.end() ||
      std::next(overlap_it)->path_idx != overlap_it->path_idx + 1) {
      for (const auto & open_interval : opened_intervals) {
        Interval interval;
        interval.lane_id = open_interval.lane_id;
        interval.entering_path_idx = open_interval.index;
        // TODO(Maxime): this may go beyond the last path point
        interval.exiting_path_idx = overlap_it->path_idx + 1;
        intervals.push_back(interval);
      }
      opened_intervals.clear();
    } else {
      for (auto it = opened_intervals.begin(); it != opened_intervals.end();) {
        if (!it->should_keep_open) {
          Interval interval;
          interval.lane_id = it->lane_id;
          interval.entering_path_idx = it->index;
          interval.exiting_path_idx = overlap_it->path_idx;
          intervals.push_back(interval);
          it = opened_intervals.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
  return intervals;
}

inline std::vector<Slowdown> calculate_decisions(
  const Intervals & intervals, const PathWithLaneId & ego_path, const size_t first_idx,
  const PredictedObjects & objects)
{
  const auto time_along_path = [&](const auto & idx) {
    // TODO(Maxime): this is a bad estimate of the time to reach a point. should use current
    // velocity ?
    auto t = 0.0;
    for (auto i = first_idx; i <= idx && i + 1 < ego_path.points.size(); ++i) {
      const auto ds =
        tier4_autoware_utils::calcDistance2d(ego_path.points[i], ego_path.points[i + 1]);
      const auto v = ego_path.points[i].point.longitudinal_velocity_mps;
      t += ds / v;
    }
    return t;
  };
  const auto object_time_to_point = [&](const auto & object, const auto & idx) {
    // TODO(Maxime): this is a bad estimate of the time to reach a point. should use lanelet map
    const auto dist = tier4_autoware_utils::calcDistance2d(
      ego_path.points[idx], object.kinematics.initial_pose_with_covariance.pose);
    const auto v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    return dist / v;
  };
  std::vector<Slowdown> decisions;
  std::cout << "** Decisions\n";
  for (const auto & interval : intervals) {
    std::printf(
      "\t[%lu -> %lu] %ld\n", interval.entering_path_idx, interval.exiting_path_idx,
      interval.lane_id);
    // skip if we already entered the interval
    if (interval.entering_path_idx <= first_idx) continue;
    const auto ego_enter_time = time_along_path(interval.entering_path_idx);
    const auto ego_exit_time = time_along_path(interval.exiting_path_idx);
    auto min_object_enter_time = std::numeric_limits<double>::max();
    auto max_object_exit_time = 0.0;
    for (const auto & object : objects.objects) {
      const auto min_vel = 0.1;  // TODO(Maxime): param
      if (object.kinematics.initial_twist_with_covariance.twist.linear.x < min_vel) continue;
      const auto enter_time = object_time_to_point(object, interval.entering_path_idx);
      const auto exit_time = object_time_to_point(
        object, std::min(ego_path.points.size() - 1, interval.exiting_path_idx));
      std::printf(
        "\t\t[%s] going at %2.2fm/s enter at %2.2fs, exits at %2.2fs\n",
        tier4_autoware_utils::toHexString(object.object_id).c_str(),
        object.kinematics.initial_twist_with_covariance.twist.linear.x, enter_time, exit_time);
      if (enter_time < std::numeric_limits<double>::infinity()) {
        min_object_enter_time = std::min(enter_time, exit_time);
        max_object_exit_time = std::max(exit_time, enter_time);
      }
    }
    // TODO(Maxime): more complex decisions, threshold, slowdown instead of stop
    constexpr auto use_threshold = true;
    const auto ego_enters_before_object = ego_enter_time < min_object_enter_time;
    const auto ego_exits_after_object = ego_exit_time > max_object_exit_time;
    // TODO(Maxime): param
    const auto should_stop = (use_threshold && min_object_enter_time < 3.0) ||
                             (!use_threshold && ego_enters_before_object && ego_exits_after_object);
    if (should_stop) {
      Slowdown decision;
      decision.target_path_idx = first_idx + ego_enter_time;
      decision.velocity = 0.0;
      decisions.push_back(decision);
    }
  }
  return decisions;
}
}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
