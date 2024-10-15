// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/type_alias.hpp"
#include "static_obstacle_avoidance_parameters.hpp"

#include <rclcpp/time.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;
using autoware::route_handler::Direction;

enum class ObjectInfo {
  NONE = 0,
  // ignore reasons
  OUT_OF_TARGET_AREA,
  FURTHER_THAN_THRESHOLD,
  FURTHER_THAN_GOAL,
  IS_NOT_TARGET_OBJECT,
  IS_NOT_PARKING_OBJECT,
  TOO_NEAR_TO_CENTERLINE,
  TOO_NEAR_TO_GOAL,
  MOVING_OBJECT,
  UNSTABLE_OBJECT,
  CROSSWALK_USER,
  ENOUGH_LATERAL_DISTANCE,
  LESS_THAN_EXECUTION_THRESHOLD,
  PARALLEL_TO_EGO_LANE,
  MERGING_TO_EGO_LANE,
  DEVIATING_FROM_EGO_LANE,
  // unavoidable reasons
  NEED_DECELERATION,
  SAME_DIRECTION_SHIFT,
  LIMIT_DRIVABLE_SPACE_TEMPORARY,
  INSUFFICIENT_DRIVABLE_SPACE,
  INSUFFICIENT_LONGITUDINAL_DISTANCE,
  INVALID_SHIFT_LINE,
  // others
  AMBIGUOUS_STOPPED_VEHICLE,
};

struct ObjectParameter
{
  bool is_avoidance_target{false};

  bool is_safety_check_target{false};

  double moving_speed_threshold{0.0};

  double moving_time_threshold{1.0};

  double max_expand_ratio{0.0};

  double envelope_buffer_margin{0.0};

  double lateral_soft_margin{1.0};

  double lateral_hard_margin{1.0};

  double lateral_hard_margin_for_parked_vehicle{1.0};

  double longitudinal_margin{0.0};

  double th_error_eclipse_long_radius{0.0};
};

struct AvoidanceParameters : public ::static_obstacle_avoidance::Params
{
  // parameters depend on object class
  std::unordered_map<uint8_t, ObjectParameter> object_parameters;

  // ego predicted path params.
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};

  // rss parameters
  utils::path_safety_checker::RSSparams rss_params{};

  AvoidanceParameters & operator=(const ::static_obstacle_avoidance::Params & params)
  {
    ::static_obstacle_avoidance::Params::operator=(params);
    return *this;
  }
};

struct ObjectData  // avoidance target
{
  ObjectData() = default;

  ObjectData(PredictedObject obj, double lat, double lon, double len)
  : object(std::move(obj)), to_centerline(lat), longitudinal(lon), length(len)
  {
  }

  Pose getPose() const { return object.kinematics.initial_pose_with_covariance.pose; }

  Point getPosition() const { return object.kinematics.initial_pose_with_covariance.pose.position; }

  PredictedObject object;

  // object behavior.
  enum class Behavior {
    NONE = 0,
    MERGING,
    DEVIATING,
  };
  Behavior behavior{Behavior::NONE};

  // lateral position of the CoM, in Frenet coordinate from ego-pose

  double to_centerline{0.0};

  // longitudinal position of the CoM, in Frenet coordinate from ego-pose
  double longitudinal{0.0};

  // longitudinal length of vehicle, in Frenet coordinate
  double length{0.0};

  // lateral shiftable ratio
  double shiftable_ratio{0.0};

  // distance factor for perception noise (0.0~1.0)
  double distance_factor{0.0};

  // count up when object disappeared. Removed when it exceeds threshold.
  rclcpp::Time last_seen{rclcpp::Clock(RCL_ROS_TIME).now()};
  double lost_time{0.0};

  // count up when object moved. Removed when it exceeds threshold.
  rclcpp::Time last_stop;
  double move_time{0.0};

  // object stopping duration
  rclcpp::Time last_move;
  double stop_time{0.0};

  // It is one of the ego driving lanelets (closest lanelet to the object) and used in the logic to
  // check whether the object is on the ego lane.
  lanelet::ConstLanelet overhang_lanelet;

  // the position at the detected moment
  Pose init_pose;

  // envelope polygon
  Polygon2d envelope_poly{};

  // envelope polygon centroid
  Point2d centroid{};

  // lateral distance from overhang to the road shoulder
  double to_road_shoulder_distance{0.0};

  // to intersection
  double to_stop_factor_distance{std::numeric_limits<double>::infinity()};

  // to stop line distance
  double to_stop_line{std::numeric_limits<double>::infinity()};

  // long radius of the covariance error ellipse
  double error_eclipse_max{std::numeric_limits<double>::infinity()};

  // if lateral margin is NOT enough, the ego must avoid the object.
  bool avoid_required{false};

  // is avoidable by behavior module
  bool is_avoidable{false};

  // is stoppable under the constraints
  bool is_stoppable{false};

  // is within intersection area
  bool is_within_intersection{false};

  // is parked vehicle on road shoulder
  bool is_parked{false};

  // is driving on ego current lane
  bool is_on_ego_lane{false};

  // is ambiguous stopped vehicle.
  bool is_ambiguous{false};

  // is clip targe.
  bool is_clip_target{false};

  // object direction.
  Direction direction{Direction::NONE};

  // overhang points (sort by distance)
  std::vector<std::pair<double, Point>> overhang_points{};

  // object detail info
  ObjectInfo info{ObjectInfo::NONE};

  // lateral avoid margin
  std::optional<double> avoid_margin{std::nullopt};

  // the nearest bound point (use in road shoulder distance calculation)
  std::optional<std::pair<Point, Point>> narrowest_place{std::nullopt};
};
using ObjectDataArray = std::vector<ObjectData>;

/*
 * Shift point with additional info for avoidance planning
 */
struct AvoidLine : public ShiftLine
{
  // object side
  bool object_on_right = true;

  // Distance from ego to start point in Frenet
  double start_longitudinal = 0.0;

  // Distance from ego to end point in Frenet
  double end_longitudinal = 0.0;

  // for the case the point is created by merge other points
  std::vector<UUID> parent_ids{};

  // corresponding object
  ObjectData object{};

  double getRelativeLength() const { return end_shift_length - start_shift_length; }

  double getRelativeLongitudinal() const { return end_longitudinal - start_longitudinal; }

  double getGradient() const { return getRelativeLength() / getRelativeLongitudinal(); }
};
using AvoidLineArray = std::vector<AvoidLine>;

struct AvoidOutline
{
  AvoidOutline(AvoidLine avoid_line, const std::optional<AvoidLine> return_line)
  : avoid_line{std::move(avoid_line)}, return_line{std::move(return_line)}
  {
  }

  AvoidLine avoid_line{};

  std::optional<AvoidLine> return_line{};

  AvoidLineArray middle_lines{};
};
using AvoidOutlines = std::vector<AvoidOutline>;

/*
 * avoidance state
 */
enum class AvoidanceState {
  RUNNING = 0,
  CANCEL,
  SUCCEEDED,
};

/*
 * Common data for avoidance planning
 */
struct AvoidancePlanningData
{
  // ego final state
  AvoidanceState state{AvoidanceState::RUNNING};

  // un-shifted pose (for current lane detection)
  Pose reference_pose;

  // reference path (before shifting)
  PathWithLaneId reference_path;

  // reference path (pre-resampled reference path)
  PathWithLaneId reference_path_rough;

  // closest reference_path index for reference_pose
  size_t ego_closest_path_index;

  // arclength vector of the reference_path from ego.
  // If the point is behind ego_pose, the value is negative.
  std::vector<double> arclength_from_ego;

  // current driving lanelet
  lanelet::ConstLanelets current_lanelets;
  lanelet::ConstLanelets extend_lanelets;

  // output path
  ShiftedPath candidate_path;

  // avoidance target objects
  ObjectDataArray target_objects;

  // the others
  ObjectDataArray other_objects;

  // nearest object that should be avoid
  std::optional<ObjectData> stop_target_object{std::nullopt};

  std::optional<lanelet::ConstLanelet> red_signal_lane{std::nullopt};

  // new shift point
  AvoidLineArray new_shift_line{};

  // safe shift point
  AvoidLineArray safe_shift_line{};

  std::vector<DrivableLanes> drivable_lanes{};

  std::vector<Point> right_bound{};

  std::vector<Point> left_bound{};

  bool safe{false};

  bool valid{false};

  bool ready{false};

  bool comfortable{false};

  bool avoid_required{false};

  bool yield_required{false};

  bool found_avoidance_path{false};

  bool force_deactivated{false};

  double to_stop_line{std::numeric_limits<double>::max()};

  double to_start_point{std::numeric_limits<double>::lowest()};

  double to_return_point{std::numeric_limits<double>::max()};
};

/*
 * Data struct for shift line generation
 */
struct ShiftLineData
{
  std::vector<double> shift_line;

  std::vector<double> pos_shift_line;

  std::vector<double> neg_shift_line;

  std::vector<double> shift_line_grad;

  std::vector<double> pos_shift_line_grad;

  std::vector<double> neg_shift_line_grad;

  std::vector<double> forward_grad;

  std::vector<double> backward_grad;

  std::vector<std::vector<double>> shift_line_history;
};

/*
 * Debug information for marker array
 */
struct DebugData
{
  std::vector<geometry_msgs::msg::Polygon> detection_areas;

  lanelet::ConstLineStrings3d bounds;

  // combine process
  AvoidLineArray step1_registered_shift_line;
  AvoidLineArray step1_current_shift_line;
  AvoidLineArray step1_filled_shift_line;
  AvoidLineArray step1_merged_shift_line;
  AvoidLineArray step1_combined_shift_line;
  AvoidLineArray step1_return_shift_line;
  AvoidLineArray step1_front_shift_line;

  // create outline process
  AvoidLineArray step2_merged_shift_line;

  // trimming process
  AvoidLineArray step3_quantize_filtered;
  AvoidLineArray step3_noise_filtered;
  AvoidLineArray step3_grad_filtered_1st;
  AvoidLineArray step3_grad_filtered_2nd;
  AvoidLineArray step3_grad_filtered_3rd;

  // registered process
  AvoidLineArray step4_new_shift_line;

  // shift length
  std::vector<double> pos_shift;
  std::vector<double> neg_shift;
  std::vector<double> total_shift;
  std::vector<double> output_shift;

  // shift grad
  std::vector<double> pos_shift_grad;
  std::vector<double> neg_shift_grad;
  std::vector<double> total_forward_grad;
  std::vector<double> total_backward_grad;

  // shift path
  std::vector<double> proposed_spline_shift;

  // avoidance require objects
  ObjectDataArray unavoidable_objects;

  // avoidance unsafe objects
  ObjectDataArray unsafe_objects;

  // tmp for plot
  PathWithLaneId center_line;

  // safety check area
  lanelet::ConstLanelets safety_check_lanes;

  // collision check debug map
  utils::path_safety_checker::CollisionCheckDebugMap collision_check;

  // debug msg array
  AvoidanceDebugMsgArray avoidance_debug_msg_array;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
