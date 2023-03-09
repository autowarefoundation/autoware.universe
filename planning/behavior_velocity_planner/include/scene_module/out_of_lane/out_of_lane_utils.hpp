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
#include <route_handler/route_handler.hpp>
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
#include <fstream>
#include <limits>
#include <list>
#include <memory>
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
  bool use_threshold;  // if true, use a time threshold to decide to stop before going out of lane

  double objects_min_vel;   // [m/s] objects lower than this velocity will be ignored
  double objects_time_thr;  // if using threshold, stop for an object whose incoming time is below
                            // this value
  double objects_slow_time_thr;  // if using threshold, slowdown for an object whose incoming time
                                 // is below this value

  double overlap_extra_length;  // [m] extra length to add around an overlap interval
  double overlap_min_dist;      // [m] min distance inside a lane for a footprint to be considered
                                // overlapping

  double slow_velocity;
  double stop_extra_dist;
  double stop_max_decel;
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
  lanelet::ConstLanelet lane_to_avoid;
};

struct Interval
{
  lanelet::ConstLanelet lane;
  size_t entering_path_idx;
  size_t exiting_path_idx;
  lanelet::BasicPoint2d entering_point;  // pose of the overlapping point closest along the lane
  lanelet::BasicPoint2d exiting_point;   // pose of the overlapping point furthest along the lane
  double inside_distance;                // [m] how much ego footprint enters the lane
};

typedef std::vector<Interval> Intervals;

struct DebugData
{
  std::vector<lanelet::BasicPolygon2d> footprints;
  Intervals intervals;
  std::vector<Pose> slowdown_poses;
  void resetData()
  {
    footprints.clear();
    intervals.clear();
    slowdown_poses.clear();
  }
};

inline tier4_autoware_utils::Polygon2d make_base_footprint(const PlannerParam & p)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  base_footprint.outer() = {
    {p.front_offset + p.extra_front_offset, p.left_offset + p.extra_left_offset},
    {p.front_offset + p.extra_front_offset, p.right_offset - p.extra_right_offset},
    {p.rear_offset - p.extra_rear_offset, p.right_offset - p.extra_right_offset},
    {p.rear_offset - p.extra_rear_offset, p.left_offset + p.extra_left_offset}};
  return base_footprint;
}

inline lanelet::BasicPolygon2d project_to_pose(
  const tier4_autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
  BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

/// @brief calculate the path footprints
/// @details the resulting polygon follows the format used by the lanelet library: clockwise order
/// and implicit closing edge
inline std::vector<BasicPolygon2d> calculate_path_footprints(
  const PathWithLaneId & path, const size_t first_idx, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
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

inline Intervals calculate_overlapping_intervals(
  const std::vector<BasicPolygon2d> & path_footprints,
  [[maybe_unused]] const lanelet::ConstLanelets & path_lanelets,
  [[maybe_unused]] const size_t idx_offset, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params)
{
  struct IntervalBound
  {
    size_t index;
    lanelet::BasicPoint2d point;
    double arc_length;
    double inside_distance;
  };
  struct OtherLane
  {
    bool interval_is_open = false;
    IntervalBound first_interval_bound{};
    IntervalBound last_interval_bound{};  // TODO(Maxime): if we look ahead we do not need this
    lanelet::ConstLanelet lanelet;
    lanelet::BasicPolygon2d polygon;

    explicit OtherLane(lanelet::ConstLanelet ll) : lanelet(ll)
    {
      polygon = lanelet.polygon2d().basicPolygon();
    }

    [[nodiscard]] Interval closeInterval()
    {
      Interval interval;
      interval.lane = lanelet;
      interval.entering_path_idx = first_interval_bound.index;
      interval.entering_point = first_interval_bound.point;
      interval.exiting_path_idx = last_interval_bound.index;
      interval.exiting_point = last_interval_bound.point;
      interval.inside_distance =
        std::max(first_interval_bound.inside_distance, last_interval_bound.inside_distance);
      interval_is_open = false;
      last_interval_bound = {};
      return interval;
    }
  };

  std::vector<OtherLane> other_lanes;
  for (const auto & lanelet : lanelets) other_lanes.emplace_back(lanelet);

  Intervals intervals;

  std::ofstream svg("/tmp/debug.svg");
  boost::geometry::svg_mapper<lanelet::BasicPoint2d> mapper(svg, 800, 800);
  for (const auto & path_footprint : path_footprints) mapper.add(path_footprint);

  for (auto i = 0UL; i < path_footprints.size(); ++i) {
    auto path_footprint = path_footprints[i];
    mapper.map(path_footprint, "opacity:0.5;fill-opacity:0.0;fill:red;stroke:red;stroke-width:1");
    for (auto & other_lane : other_lanes) {
      lanelet::BasicPolygons2d overlapping_polygons;
      boost::geometry::intersection(path_footprint, other_lane.polygon, overlapping_polygons);
      auto inside_dist = 0.0;  // maximize
      lanelet::BasicPoint2d min_overlap_point;
      lanelet::BasicPoint2d max_overlap_point;
      auto min_arc_length = std::numeric_limits<double>::infinity();
      auto max_arc_length = 0.0;
      for (const auto & overlapping_polygon : overlapping_polygons) {
        mapper.map(
          overlapping_polygon, "opacity:0.5;fill-opacity:0.2;fill:red;stroke:red;stroke-width:1");
        for (const auto & point : overlapping_polygon) {
          // TODO(Maxime): inside distance is not correct (need to use the intersected left/right
          // bound of the lanelet)
          inside_dist = std::max(inside_dist, boost::geometry::distance(point, other_lane.polygon));
          const auto length =
            lanelet::geometry::toArcCoordinates(other_lane.lanelet.centerline2d(), point).length;
          if (length > max_arc_length) {
            max_arc_length = length;
            max_overlap_point = point;
          }
          if (length < min_arc_length) {
            min_arc_length = length;
            min_overlap_point = point;
          }
        }
      }
      if (!overlapping_polygons.empty()) {  // if there was an overlap, open/update the interval
        if (!other_lane.interval_is_open) {
          other_lane.first_interval_bound.index = i;
          other_lane.first_interval_bound.point = min_overlap_point;
          other_lane.first_interval_bound.arc_length = min_arc_length - params.overlap_extra_length;
          other_lane.first_interval_bound.inside_distance = inside_dist;
          other_lane.interval_is_open = true;
        }
        other_lane.last_interval_bound.index = i;
        other_lane.last_interval_bound.point = max_overlap_point;
        other_lane.last_interval_bound.arc_length = max_arc_length + params.overlap_extra_length;
        other_lane.last_interval_bound.inside_distance = inside_dist;
      } else if (other_lane.interval_is_open) {  // close the interval if there were no overlap
        intervals.push_back(other_lane.closeInterval());
      }
    }
  }
  // close all open intervals
  for (auto & other_lane : other_lanes)
    if (other_lane.interval_is_open) {
      intervals.push_back(other_lane.closeInterval());
    }
  return intervals;
}

inline std::vector<Slowdown> calculate_decisions(
  const Intervals & intervals, const PathWithLaneId & ego_path, const size_t first_idx,
  const PredictedObjects & objects, std::shared_ptr<route_handler::RouteHandler> route_handler,
  const lanelet::ConstLanelets & lanelets, const PlannerParam & params)
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
  const auto object_time_to_interval = [&](const auto & object, const auto & interval) {
    const auto & p = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto object_point = lanelet::BasicPoint2d(p.x, p.y);
    lanelet::ConstLanelets object_lanelets;
    for (const auto & ll : lanelets)
      if (boost::geometry::within(object_point, ll.polygon2d().basicPolygon()))
        object_lanelets.push_back(ll);
    auto min_dist = std::numeric_limits<double>::infinity();
    auto max_dist = 0.0;
    for (const auto & lane : object_lanelets) {
      const auto path = route_handler->getRoutingGraphPtr()->shortestPath(lane, interval.lane);
      std::printf(
        "\t\t\tPath ? %d [from %ld to %ld]\n", path.has_value(), lane.id(), interval.lane.id());
      if (path) {
        lanelet::ConstLanelets lls;
        for (const auto & ll : *path) lls.push_back(ll);
        geometry_msgs::msg::Pose p;
        p.position.x = object_point.x();
        p.position.y = object_point.y();
        const auto object_length = lanelet::utils::getArcCoordinates(lls, p).length;
        p.position.x = interval.entering_point.x();
        p.position.y = interval.entering_point.y();
        const auto enter_length = lanelet::utils::getArcCoordinates(lls, p).length;
        p.position.x = interval.exiting_point.x();
        p.position.y = interval.exiting_point.y();
        const auto exit_length = lanelet::utils::getArcCoordinates(lls, p).length;
        std::printf(
          "\t\t\t%2.2f -> [%2.2f(%2.2f, %2.2f) - %2.2f(%2.2f, %2.2f)]\n", object_length,
          enter_length, interval.entering_point.x(), interval.entering_point.y(), exit_length,
          interval.exiting_point.x(), interval.exiting_point.y());
        const auto & [object_enter_length, object_exit_length] =
          std::minmax(enter_length, exit_length);
        min_dist = std::min(min_dist, object_enter_length - object_length);
        max_dist = std::max(max_dist, object_exit_length - object_length);
      }
    }
    const auto v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    return std::make_pair(min_dist / v, max_dist / v);
  };
  std::vector<Slowdown> decisions;
  std::cout << "** Decisions\n";
  for (const auto & interval : intervals) {
    // skip if we already entered the interval
    if (interval.entering_path_idx == 0UL) continue;
    // skip if the overlap inside the interval is too small
    if (interval.inside_distance < params.overlap_min_dist) continue;

    const auto ego_enter_time = time_along_path(interval.entering_path_idx);
    const auto ego_exit_time = time_along_path(interval.exiting_path_idx);
    std::printf(
      "\t[%lu -> %lu] %ld (ego enters at %2.2f, exits at %2.2f)\n", interval.entering_path_idx,
      interval.exiting_path_idx, interval.lane.id(), ego_enter_time, ego_exit_time);
    auto min_object_enter_time = std::numeric_limits<double>::max();
    auto max_object_exit_time = 0.0;
    for (const auto & object : objects.objects) {
      if (object.kinematics.initial_twist_with_covariance.twist.linear.x < params.objects_min_vel)
        continue;  // skip objects with velocity bellow a threshold
      const auto & [enter_time, exit_time] = object_time_to_interval(object, interval);
      std::printf(
        "\t\t[%s] going at %2.2fm/s enter at %2.2fs, exits at %2.2fs\n",
        tier4_autoware_utils::toHexString(object.object_id).c_str(),
        object.kinematics.initial_twist_with_covariance.twist.linear.x, enter_time, exit_time);
      min_object_enter_time = std::min(min_object_enter_time, std::max(0.0, enter_time));
      max_object_exit_time = std::max(max_object_exit_time, exit_time);
    }
    // TODO(Maxime): more complex decisions, threshold, slowdown instead of stop
    const auto incoming_objects = min_object_enter_time < max_object_exit_time;
    const auto ego_enters_before_object = ego_enter_time < min_object_enter_time;
    const auto ego_exits_after_object = ego_exit_time > max_object_exit_time;
    const auto threshold_stop_condition = min_object_enter_time < params.objects_time_thr;
    const auto threshold_slow_condition = min_object_enter_time < params.objects_slow_time_thr;
    const auto interval_stop_condition = ego_enters_before_object && ego_exits_after_object;
    const auto should_stop =
      incoming_objects && ((params.use_threshold && threshold_stop_condition) ||
                           (!params.use_threshold && interval_stop_condition));
    const auto should_slow = incoming_objects && params.use_threshold && threshold_slow_condition;
    Slowdown decision;
    decision.target_path_idx = first_idx + interval.entering_path_idx;
    decision.lane_to_avoid = interval.lane;
    if (should_stop) {
      std::printf("\t\tWill stop\n");
      decision.velocity = 0.0;
      decisions.push_back(decision);
    } else if (should_slow) {
      std::printf("\t\tWill slow\n");
      decision.velocity = params.slow_velocity;
      decisions.push_back(decision);
    }
  }
  return decisions;
}

inline void insert_slowdown_points(
  PathWithLaneId & path, const std::vector<Slowdown> & decisions, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
  for (const auto & decision : decisions) {
    const auto & path_point = path.points[decision.target_path_idx];
    auto path_idx = decision.target_path_idx;
    if (decision.target_path_idx == 0) {
      planning_utils::insertVelocity(path, path_point, decision.velocity, path_idx);
      return;  // TODO(Maxime): should we insert more than the first decision ?
    } else {
      const auto & path_pose = path_point.point.pose;
      const auto & prev_path_pose = path.points[decision.target_path_idx - 1].point.pose;

      const auto precision = 0.1;  // TODO(Maxime): param or better way to find no overlap pose
      auto interpolated_point = path_point;
      for (auto ratio = precision; ratio <= 1.0; ratio += precision) {
        interpolated_point.point.pose =
          tier4_autoware_utils::calcInterpolatedPose(path_pose, prev_path_pose, ratio, false);
        const auto overlaps = boost::geometry::overlaps(
          project_to_pose(base_footprint, interpolated_point.point.pose),
          decision.lane_to_avoid.polygon2d().basicPolygon());
        if (!overlaps) {
          planning_utils::insertVelocity(path, interpolated_point, decision.velocity, path_idx);
          return;  // TODO(Maxime): should we insert more than the first decision ?
        }
      }
    }
  }
}
}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
