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

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <limits>
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
  double front_offset;  // [m]  front offset
  double rear_offset;   // [m]  rear offset
  double right_offset;  // [m]  right offset
  double left_offset;   // [m]  left offset
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
};

struct Interval
{
  size_t entering_path_idx;
  size_t exiting_path_idx;
  lanelet::Ids overlapped_lanes;
};

typedef std::vector<Overlap> Overlaps;
typedef std::vector<Interval> Intervals;

struct DebugData
{
  std::vector<lanelet::BasicPolygon2d> footprints;
  Overlaps overlaps;
  Intervals intervals;
  void resetData()
  {
    footprints.clear();
    overlaps.clear();
    intervals.clear();
  }
};

inline std::vector<BasicPolygon2d> calculate_path_footprints(
  const PathWithLaneId & path, const PlannerParam & params)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  base_footprint.outer() = {
    {params.front_offset, params.right_offset},
    {params.front_offset, params.left_offset},
    {params.rear_offset, params.left_offset},
    {params.rear_offset, params.right_offset}};

  std::vector<BasicPolygon2d> path_footprints;
  path_footprints.reserve(path.points.size());
  for (const auto & path_point : path.points) {
    const auto & path_pose = path_point.point.pose;
    const auto angle = tf2::getYaw(path_pose.orientation);
    const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
    BasicPolygon2d footprint;
    for (const auto & p : rotated_footprint.outer())
      footprint.emplace_back(p.x() + path_pose.position.x, p.y() + path_pose.position.y);
    footprint.push_back(footprint.front());  // close the polygon
    path_footprints.push_back(footprint);
  }
  return path_footprints;
}

inline Overlaps calculate_overlaps(
  const std::vector<BasicPolygon2d> & path_footprints, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params)
{
  Overlaps overlaps;
  (void)params;
  // TODO(Maxime): only check overlaps if the left/right bound of the path lanelets are crossed.
  for (auto i = 0UL; i < path_footprints.size(); ++i) {
    const auto & path_point_polygon = path_footprints[i];
    Overlap overlap;
    overlap.path_idx = i;
    for (const auto & lanelet : lanelets) {
      const auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
      auto inside_dist = 0.0;
      auto arc_length_along_lanelet = std::numeric_limits<double>::infinity();
      for (const auto & lanelet_bound :
           {lanelet.leftBound2d().basicLineString(), lanelet.rightBound2d().basicLineString()}) {
        lanelet::BasicPoints2d intersections;
        boost::geometry::intersection(path_point_polygon, lanelet_bound, intersections);
        if (!intersections.empty()) {
          for (const auto & p : path_point_polygon) {
            if (boost::geometry::within(p, lanelet_polygon)) {
              inside_dist = std::min(inside_dist, boost::geometry::distance(p, lanelet_bound));
              arc_length_along_lanelet = std::min(
                arc_length_along_lanelet,
                lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), p).length);
            }
          }
          for (const auto & intersection : intersections) {
            arc_length_along_lanelet = std::min(
              arc_length_along_lanelet,
              lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), intersection).length);
          }
        }
      }
      if (arc_length_along_lanelet < std::numeric_limits<double>::infinity()) {
        overlap.overlapped_lanes.push_back(lanelet.id());
        overlap.overlap_distances.push_back(inside_dist);
        overlap.overlap_arc_lengths.push_back(arc_length_along_lanelet);
      }
    }
    if (!overlap.overlapped_lanes.empty()) overlaps.push_back(overlap);
  }
  return overlaps;
}

inline Intervals calculate_overlapping_intervals()
{
  Intervals intervals;
  return intervals;
}

inline std::vector<Slowdown> calculate_decisions(const Intervals & intervals)
{
  std::vector<Slowdown> decisions;
  (void)intervals;
  return decisions;
}
}  // namespace out_of_lane_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OUT_OF_LANE__OUT_OF_LANE_UTILS_HPP_
