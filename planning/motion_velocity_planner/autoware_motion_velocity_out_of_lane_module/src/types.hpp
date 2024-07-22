// Copyright 2024 TIER IV, Inc.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware/motion_velocity_planner_common/ttc_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/geometries/multi_polygon.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{
using Polygons = boost::geometry::model::multi_polygon<lanelet::BasicPolygonWithHoles2d>;

/// @brief parameters for the out_of_lane module
struct PlannerParam
{
  std::string mode;                  // mode used to consider a conflict with an object
  bool skip_if_already_overlapping;  // if true, do not run the module when ego already overlaps
                                     // another lane

  double time_threshold;        // [s](mode="threshold") objects time threshold
  double intervals_ego_buffer;  // [s](mode="intervals") buffer to extend the ego time range
  double intervals_obj_buffer;  // [s](mode="intervals") buffer to extend the objects time range
  double ttc_threshold;  // [s](mode="ttc") threshold on time to collision between ego and an object
  double ego_min_velocity;  // [m/s] minimum velocity of ego used to calculate its ttc or time range

  bool objects_use_predicted_paths;  // whether to use the objects' predicted paths
  bool objects_cut_predicted_paths_beyond_red_lights;  // whether to cut predicted paths beyond red
                                                       // lights' stop lines
  double objects_min_vel;         // [m/s] objects lower than this velocity will be ignored
  double objects_min_confidence;  // minimum confidence to consider a predicted path
  double objects_dist_buffer;  // [m] distance buffer used to determine if a collision will occur in
                               // the other lane
  bool objects_ignore_behind_ego;  // if true, objects behind the ego vehicle are ignored

  double overlap_extra_length;  // [m] extra length to add around an overlap range
  double overlap_min_dist;      // [m] min distance inside another lane to consider an overlap
  // action to insert in the trajectory if an object causes a conflict at an overlap
  bool skip_if_over_max_decel;  // if true, skip the action if it causes more than the max decel
  double lon_dist_buffer;       // [m] safety distance buffer to keep in front of the ego vehicle
  double lat_dist_buffer;       // [m] safety distance buffer to keep on the side of the ego vehicle
  double slow_velocity;
  double stop_dist_threshold;
  double precision;              // [m] precision when inserting a stop pose in the trajectory
  double min_decision_duration;  // [s] minimum duration needed a decision can be canceled
  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance

  bool use_route_to_get_route_lanelets = true;  // TODO(Maxime): param
  double max_arc_length = 100.0;                // TODO(Maxime): param
};

/// @brief data related to the ego vehicle
struct EgoData
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory_points;
  geometry_msgs::msg::Pose pose;
  size_t first_trajectory_idx{};
  double longitudinal_offset_to_first_trajectory_index{};
  double min_stop_distance{};
  double min_slowdown_distance{};

  Polygons drivable_lane_polygons;

  lanelet::BasicPolygon2d current_footprint;
  std::vector<lanelet::BasicPolygon2d> trajectory_footprints;

  lanelet::ConstLanelet ego_lanelet;
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets ignored_lanelets;
  lanelet::ConstLanelets out_of_lane_lanelets;
};

struct OutOfLanePoint
{
  size_t trajectory_index;
  lanelet::BasicPolygons2d outside_rings;
  std::set<double> collision_times;
  std::optional<double> min_object_arrival_time;
  std::optional<double> max_object_arrival_time;
  std::optional<double> ttc;
  bool to_avoid = false;
};
struct OutOfLaneData
{
  std::vector<OutOfLanePoint> outside_points;
  lanelet::ConstLanelets out_of_lane_lanelets;
};

/// @brief debug data
struct DebugData
{
  size_t prev_footprints = 0;
  size_t prev_route_lanelets = 0;
  size_t prev_ignored_lanelets = 0;
  size_t prev_out_of_lane_lanelets = 0;
  size_t prev_drivable_lane_polygons = 0;
  size_t prev_out_of_lane_areas = 0;
  size_t prev_ttcs = 0;
};

}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // TYPES_HPP_
