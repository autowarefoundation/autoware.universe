// Copyright 2020-2023 Tier IV, Inc.
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

#include "scene_crosswalk.hpp"

#include "occluded_crosswalk.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;
using autoware::motion_utils::calcArcLength;
using autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints;
using autoware::motion_utils::calcLateralOffset;
using autoware::motion_utils::calcLongitudinalOffsetPoint;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::calcSignedArcLengthPartialSum;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::motion_utils::resamplePath;
using autoware::universe_utils::createPoint;
using autoware::universe_utils::getPose;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;
using autoware::universe_utils::toHexString;

namespace
{
geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<geometry_msgs::msg::Point> toGeometryPointVector(
  const Polygon2d & polygon, const double z)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & p : polygon.outer()) {
    points.push_back(createPoint(p.x(), p.y(), z));
  }
  return points;
}

void offsetPolygon2d(
  const geometry_msgs::msg::Pose & origin_point, const geometry_msgs::msg::Polygon & polygon,
  Polygon2d & offset_polygon)
{
  for (const auto & polygon_point : polygon.points) {
    const auto offset_pos =
      autoware::universe_utils::calcOffsetPose(origin_point, polygon_point.x, polygon_point.y, 0.0)
        .position;
    offset_polygon.outer().push_back(Point2d(offset_pos.x, offset_pos.y));
  }
}

template <class T>
Polygon2d createMultiStepPolygon(
  const T & obj_path_points, const geometry_msgs::msg::Polygon & polygon, const size_t start_idx,
  const size_t end_idx)
{
  Polygon2d multi_step_polygon{};
  for (size_t i = start_idx; i <= end_idx; ++i) {
    offsetPolygon2d(
      autoware::universe_utils::getPose(obj_path_points.at(i)), polygon, multi_step_polygon);
  }

  Polygon2d hull_multi_step_polygon{};
  bg::convex_hull(multi_step_polygon, hull_multi_step_polygon);
  bg::correct(hull_multi_step_polygon);

  return hull_multi_step_polygon;
}

void sortCrosswalksByDistance(
  const PathWithLaneId & ego_path, const geometry_msgs::msg::Point & ego_pos,
  lanelet::ConstLanelets & crosswalks)
{
  const auto compare = [&](const lanelet::ConstLanelet & l1, const lanelet::ConstLanelet & l2) {
    const auto l1_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l1.polygon2d().basicPolygon(), ego_pos);
    const auto l2_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l2.polygon2d().basicPolygon(), ego_pos);

    if (!l1_end_points_on_crosswalk || !l2_end_points_on_crosswalk) {
      return true;
    }

    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), l1_end_points_on_crosswalk->first);

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), l2_end_points_on_crosswalk->first);

    return dist_l1 < dist_l2;
  };

  std::sort(crosswalks.begin(), crosswalks.end(), compare);
}

std::vector<Polygon2d> calcOverlappingPoints(const Polygon2d & polygon1, const Polygon2d & polygon2)
{
  // NOTE: If one polygon is fully inside the other polygon, the result is empty.
  std::vector<Polygon2d> intersection_polygons{};
  bg::intersection(polygon1, polygon2, intersection_polygons);
  return intersection_polygons;
}

StopFactor createStopFactor(
  const geometry_msgs::msg::Pose & stop_pose,
  const std::vector<geometry_msgs::msg::Point> & stop_factor_points = {})
{
  StopFactor stop_factor;
  stop_factor.stop_factor_points = stop_factor_points;
  stop_factor.stop_pose = stop_pose;
  return stop_factor;
}

autoware_internal_debug_msgs::msg::StringStamped createStringStampedMessage(
  const rclcpp::Time & now, const int64_t module_id_,
  const std::vector<std::tuple<std::string, CollisionPoint, CollisionState>> & collision_points)
{
  autoware_internal_debug_msgs::msg::StringStamped msg;
  msg.stamp = now;
  for (const auto & collision_point : collision_points) {
    std::stringstream ss;
    ss << module_id_ << "," << std::get<0>(collision_point).substr(0, 4) << ","
       << std::get<1>(collision_point).time_to_collision << ","
       << std::get<1>(collision_point).time_to_vehicle << ","
       << static_cast<int>(std::get<2>(collision_point)) << ",";
    msg.data += ss.str();
  }
  return msg;
}
}  // namespace

CrosswalkModule::CrosswalkModule(
  rclcpp::Node & node, const int64_t lane_id, const int64_t module_id,
  const std::optional<int64_t> & reg_elem_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<universe_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  module_id_(module_id),
  planner_param_(planner_param),
  use_regulatory_element_(reg_elem_id)
{
  passed_safety_slow_point_ = false;

  if (use_regulatory_element_) {
    const auto reg_elem_ptr = std::dynamic_pointer_cast<const lanelet::autoware::Crosswalk>(
      lanelet_map_ptr->regulatoryElementLayer.get(*reg_elem_id));
    stop_lines_ = reg_elem_ptr->stopLines();
    crosswalk_ = reg_elem_ptr->crosswalkLanelet();
  } else {
    const auto stop_line = getStopLineFromMap(module_id_, lanelet_map_ptr, "crosswalk_id");
    if (stop_line) {
      stop_lines_.push_back(*stop_line);
    }
    crosswalk_ = lanelet_map_ptr->laneletLayer.get(module_id);
  }

  road_ = lanelet_map_ptr->laneletLayer.get(lane_id);

  collision_info_pub_ = node.create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/debug/collision_info", 1);
}

bool CrosswalkModule::modifyPathVelocity(PathWithLaneId * path)
{
  if (path->points.size() < 2) {
    RCLCPP_DEBUG(logger_, "Do not interpolate because path size is less than 2.");
    return {};
  }

  stop_watch_.tic("total_processing_time");
  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time,
    "=========== module_id: %ld ===========", module_id_);

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto objects_ptr = planner_data_->predicted_objects;

  // Initialize debug data
  debug_data_ = DebugData(planner_data_);
  for (const auto & p : crosswalk_.polygon2d().basicPolygon()) {
    debug_data_.crosswalk_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }
  recordTime(1);

  // Calculate intersection between path and crosswalks
  const auto path_end_points_on_crosswalk =
    getPathEndPointsOnCrosswalk(*path, crosswalk_.polygon2d().basicPolygon(), ego_pos);
  if (!path_end_points_on_crosswalk) {
    return {};
  }
  const auto & first_path_point_on_crosswalk = path_end_points_on_crosswalk->first;
  const auto & last_path_point_on_crosswalk = path_end_points_on_crosswalk->second;

  // Apply safety slow down speed if defined in Lanelet2 map
  applySlowDownByLanelet2Map(*path, first_path_point_on_crosswalk, last_path_point_on_crosswalk);

  // Apply safety slow down speed if the crosswalk is occluded
  applySlowDownByOcclusion(*path, first_path_point_on_crosswalk, last_path_point_on_crosswalk);
  recordTime(2);

  // Calculate stop point with margin
  const auto default_stop_pose = getDefaultStopPose(*path, first_path_point_on_crosswalk);

  // Resample path sparsely for less computation cost
  constexpr double resample_interval = 4.0;
  const auto sparse_resample_path =
    resamplePath(*path, resample_interval, false, true, true, false);

  // Decide to stop for crosswalk users
  const auto stop_factor_for_crosswalk_users = checkStopForCrosswalkUsers(
    *path, sparse_resample_path, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
    default_stop_pose);

  // Decide to stop for stuck vehicle
  const auto stop_factor_for_stuck_vehicles = checkStopForStuckVehicles(
    sparse_resample_path, objects_ptr->objects, first_path_point_on_crosswalk,
    last_path_point_on_crosswalk, default_stop_pose);

  // Get nearest stop factor
  const auto nearest_stop_factor =
    getNearestStopFactor(*path, stop_factor_for_crosswalk_users, stop_factor_for_stuck_vehicles);
  recordTime(3);

  // Set safe or unsafe
  setSafe(!nearest_stop_factor);

  // Set distance
  // NOTE: If no stop point is inserted, distance to the virtual stop line has to be calculated.
  setDistanceToStop(*path, default_stop_pose, nearest_stop_factor);

  // plan Go/Stop
  if (isActivated()) {
    planGo(*path, nearest_stop_factor);
  } else {
    planStop(*path, nearest_stop_factor, default_stop_pose);
  }
  recordTime(4);

  const auto collision_info_msg =
    createStringStampedMessage(clock_->now(), module_id_, debug_data_.collision_points);
  collision_info_pub_->publish(collision_info_msg);

  return true;
}

// NOTE: The stop point will be the returned point with the margin.
std::optional<geometry_msgs::msg::Pose> CrosswalkModule::getDefaultStopPose(
  const PathWithLaneId & ego_path,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk) const
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // If stop lines are found in the LL2 map.
  for (const auto & stop_line : stop_lines_) {
    const auto p_stop_lines =
      getLinestringIntersects(ego_path, lanelet::utils::to2D(stop_line).basicLineString(), ego_pos);
    if (!p_stop_lines.empty()) {
      return calcLongitudinalOffsetPose(ego_path.points, p_stop_lines.front(), -base_link2front);
    }
  }

  // If stop lines are not found in the LL2 map.
  return calcLongitudinalOffsetPose(
    ego_path.points, first_path_point_on_crosswalk,
    -planner_param_.stop_distance_from_crosswalk - base_link2front);
}

std::optional<StopFactor> CrosswalkModule::checkStopForCrosswalkUsers(
  const PathWithLaneId & ego_path, const PathWithLaneId & sparse_resample_path,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose_opt)
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  // Calculate attention range for crosswalk
  const auto crosswalk_attention_range = getAttentionRange(
    sparse_resample_path, first_path_point_on_crosswalk, last_path_point_on_crosswalk);

  // Get attention area, which is ego's footprints on the crosswalk
  const auto attention_area = getAttentionArea(sparse_resample_path, crosswalk_attention_range);

  // Update object state
  // This exceptional handling should be done in update(), but is compromised by history
  const double dist_default_stop =
    default_stop_pose_opt.has_value()
      ? calcSignedArcLength(ego_path.points, ego_pos, default_stop_pose_opt->position)
      : 0.0;
  updateObjectState(
    dist_default_stop, sparse_resample_path, crosswalk_attention_range, attention_area);

  // Check pedestrian for stop
  // NOTE: first stop point and its minimum distance from ego to stop
  std::optional<double> dist_nearest_cp;
  std::vector<geometry_msgs::msg::Point> stop_factor_points;
  for (const auto & object : object_info_manager_.getObject()) {
    const auto & collision_point_opt = object.collision_point;
    if (collision_point_opt) {
      const auto & collision_point = collision_point_opt.value();
      const auto & collision_state = object.collision_state;
      if (collision_state != CollisionState::YIELD) {
        continue;
      }

      stop_factor_points.push_back(object.position);

      const auto dist_ego2cp =
        calcSignedArcLength(sparse_resample_path.points, ego_pos, collision_point.collision_point);
      if (!dist_nearest_cp || dist_ego2cp < dist_nearest_cp) {
        dist_nearest_cp = dist_ego2cp;
      }
    }
  }
  if (!dist_nearest_cp) {
    return {};
  }

  const auto decided_stop_pose_opt =
    calcStopPose(ego_path, dist_nearest_cp.value(), default_stop_pose_opt);
  if (!decided_stop_pose_opt.has_value()) {
    return {};
  }
  return createStopFactor(decided_stop_pose_opt.value(), stop_factor_points);
}

std::optional<geometry_msgs::msg::Pose> CrosswalkModule::calcStopPose(
  const PathWithLaneId & ego_path, double dist_nearest_cp,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose_opt)
{
  struct StopCandidate
  {
    geometry_msgs::msg::Pose pose;
    double dist;
  };

  const auto & p = planner_param_;
  const auto base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const double ego_vel_non_negative =
    std::max(0.0, planner_data_->current_velocity->twist.linear.x);
  const double ego_acc = planner_data_->current_acceleration->accel.accel.linear.x;

  const auto default_stop_opt = [&]() -> std::optional<StopCandidate> {
    if (!default_stop_pose_opt.has_value()) return std::nullopt;
    return StopCandidate{
      default_stop_pose_opt.value(),
      calcSignedArcLength(ego_path.points, ego_pos, default_stop_pose_opt->position)};
  }();

  const auto ped_stop_pref_opt = [&]() -> std::optional<StopCandidate> {
    const double dist =
      dist_nearest_cp - base_link2front - planner_param_.stop_distance_from_object_preferred;
    const auto pose_opt = calcLongitudinalOffsetPose(ego_path.points, ego_pos, dist);
    if (!pose_opt.has_value()) return std::nullopt;
    return StopCandidate{pose_opt.value(), dist};
  }();

  const auto without_acc_pref_stop_opt = [&]() -> std::optional<StopCandidate> {
    if (!ped_stop_pref_opt.has_value()) {
      RCLCPP_INFO(logger_, "Failure to calculate pref_stop.");
      return std::nullopt;
    } else if (default_stop_opt.has_value() && ped_stop_pref_opt->dist > default_stop_opt->dist) {
      return default_stop_opt;
    } else {
      return ped_stop_pref_opt;
    }
  }();

  const auto ped_stop_limit = [&]() -> std::optional<StopCandidate> {
    const double dist =
      dist_nearest_cp - base_link2front - planner_param_.stop_distance_from_object_limit;
    const auto pose_opt = calcLongitudinalOffsetPose(ego_path.points, ego_pos, dist);
    if (!pose_opt.has_value()) return std::nullopt;
    return StopCandidate{pose_opt.value(), dist};
  }();
  if (!ped_stop_limit.has_value()) {
    RCLCPP_WARN(
      logger_,
      "Stop is canceled. "
      "Failure to calculate stop_pose against the nearest pedestrian with a limit margin.");
    return std::nullopt;
  }

  const auto weak_brk_stop = [&]() -> std::optional<StopCandidate> {
    const auto dist_opt = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      ego_vel_non_negative, 0.0, ego_acc, p.min_acc_preferred, 10.0, p.min_jerk_preferred);
    if (!dist_opt.has_value()) return std::nullopt;
    const auto pose_opt = calcLongitudinalOffsetPose(ego_path.points, ego_pos, dist_opt.value());
    if (!pose_opt.has_value()) return std::nullopt;
    return StopCandidate{pose_opt.value(), dist_opt.value()};
  }();
  if (!weak_brk_stop.has_value()) {
    RCLCPP_ERROR(logger_, "Failure to calculate braking distance. Stop will be canceled.");
    return std::nullopt;
  }

  const auto selected_stop = [&]() {
    if (
      without_acc_pref_stop_opt.has_value() &&
      weak_brk_stop->dist < without_acc_pref_stop_opt->dist) {
      return without_acc_pref_stop_opt.value();
    } else if (weak_brk_stop->dist < ped_stop_limit->dist) {
      return weak_brk_stop.value();
    } else {
      return ped_stop_limit.value();
    }
  }();

  const double strong_brk_dist = [&]() {
    const auto strong_brk_dist_opt = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
      ego_vel_non_negative, 0.0, ego_acc, p.min_acc_for_no_stop_decision, 10.0,
      p.min_jerk_for_no_stop_decision);
    return strong_brk_dist_opt ? strong_brk_dist_opt.value() : 0.0;
  }();
  if (selected_stop.dist < strong_brk_dist - p.overrun_threshold_length_for_no_stop_decision) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "Abandon to stop. "
      "Can not stop against the nearest pedestrian with a specified deceleration. "
      "dist to stop: %f, braking distance: %f",
      selected_stop.dist, strong_brk_dist);
    debug_data_.pass_poses.push_back(selected_stop.pose);
    return std::nullopt;
  }

  return std::make_optional(selected_stop.pose);
}

std::pair<double, double> CrosswalkModule::getAttentionRange(
  const PathWithLaneId & ego_path, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto near_attention_range =
    calcSignedArcLength(ego_path.points, ego_pos, first_path_point_on_crosswalk) -
    planner_param_.crosswalk_attention_range;
  const auto far_attention_range =
    calcSignedArcLength(ego_path.points, ego_pos, last_path_point_on_crosswalk) +
    planner_param_.crosswalk_attention_range;

  const auto [clamped_near_attention_range, clamped_far_attention_range] =
    clampAttentionRangeByNeighborCrosswalks(ego_path, near_attention_range, far_attention_range);

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(
    std::max(0.0, clamped_near_attention_range), std::max(0.0, clamped_far_attention_range));
}

void CrosswalkModule::insertDecelPointWithDebugInfo(
  const geometry_msgs::msg::Point & stop_point, const float target_velocity,
  PathWithLaneId & output) const
{
  const auto stop_pose = planning_utils::insertDecelPoint(stop_point, output, target_velocity);
  if (!stop_pose) {
    return;
  }

  debug_data_.first_stop_pose = getPose(*stop_pose);

  if (std::abs(target_velocity) < 1e-3) {
    debug_data_.stop_poses.push_back(*stop_pose);
  } else {
    debug_data_.slow_poses.push_back(*stop_pose);
  }
}

float CrosswalkModule::calcTargetVelocity(
  const geometry_msgs::msg::Point & stop_point, const PathWithLaneId & ego_path) const
{
  const auto max_jerk = planner_param_.max_slow_down_jerk;
  const auto max_accel = planner_param_.max_slow_down_accel;
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_vel = planner_data_->current_velocity->twist.linear.x;

  if (ego_vel < planner_param_.no_relax_velocity) {
    return 0.0;
  }

  const auto ego_acc = planner_data_->current_acceleration->accel.accel.linear.x;
  const auto dist_deceleration = calcSignedArcLength(ego_path.points, ego_pos, stop_point);
  const auto feasible_velocity = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    max_jerk, max_accel, ego_acc, ego_vel, dist_deceleration);

  constexpr double margin_velocity = 0.5;  // 1.8 km/h
  return margin_velocity < feasible_velocity ? feasible_velocity : 0.0;
}

std::pair<double, double> CrosswalkModule::clampAttentionRangeByNeighborCrosswalks(
  const PathWithLaneId & ego_path, const double near_attention_range,
  const double far_attention_range)
{
  stop_watch_.tic(__func__);

  const auto & ego_pos = planner_data_->current_odometry->pose.position;

  const auto p_near = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto p_far = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (!p_near || !p_far) {
    return std::make_pair(near_attention_range, far_attention_range);
  }

  const auto near_idx = findNearestSegmentIndex(ego_path.points, p_near.value());
  const auto far_idx = findNearestSegmentIndex(ego_path.points, p_far.value()) + 1;

  std::set<int64_t> lane_ids;
  for (size_t i = near_idx; i < far_idx; ++i) {
    for (const auto & id : ego_path.points.at(i).lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets crosswalks{};
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto overall_graphs_ptr = planner_data_->route_handler_->getOverallGraphPtr();

  for (const auto & id : lane_ids) {
    const auto ll = lanelet_map_ptr->laneletLayer.get(id);
    const auto conflicting_crosswalks =
      overall_graphs_ptr->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicting_crosswalks) {
      const auto itr = std::find_if(
        crosswalks.begin(), crosswalks.end(),
        [&](const lanelet::ConstLanelet & ll) { return ll.id() == crosswalk.id(); });
      if (itr == crosswalks.end()) {
        crosswalks.push_back(crosswalk);
      }
    }
  }

  sortCrosswalksByDistance(ego_path, ego_pos, crosswalks);

  std::optional<lanelet::ConstLanelet> prev_crosswalk{std::nullopt};
  std::optional<lanelet::ConstLanelet> next_crosswalk{std::nullopt};

  if (!crosswalks.empty()) {
    for (size_t i = 0; i < crosswalks.size() - 1; ++i) {
      const auto ll_front = crosswalks.at(i);
      const auto ll_back = crosswalks.at(i + 1);

      if (ll_front.id() == module_id_ && ll_back.id() != module_id_) {
        next_crosswalk = ll_back;
      }

      if (ll_front.id() != module_id_ && ll_back.id() == module_id_) {
        prev_crosswalk = ll_front;
      }
    }
  }

  const double clamped_near_attention_range = [&]() {
    if (!prev_crosswalk) {
      return near_attention_range;
    }
    auto reverse_ego_path = ego_path;
    std::reverse(reverse_ego_path.points.begin(), reverse_ego_path.points.end());

    const auto path_end_points_on_prev_crosswalk = getPathEndPointsOnCrosswalk(
      reverse_ego_path, prev_crosswalk->polygon2d().basicPolygon(), ego_pos);
    if (!path_end_points_on_prev_crosswalk) {
      return near_attention_range;
    }

    const auto dist_to_prev_crosswalk =
      calcSignedArcLength(ego_path.points, ego_pos, path_end_points_on_prev_crosswalk->first);
    return std::max(near_attention_range, dist_to_prev_crosswalk);
  }();

  const double clamped_far_attention_range = [&]() {
    if (!next_crosswalk) {
      return far_attention_range;
    }
    const auto path_end_points_on_next_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, next_crosswalk->polygon2d().basicPolygon(), ego_pos);
    if (!path_end_points_on_next_crosswalk) {
      return far_attention_range;
    }

    const auto dist_to_next_crosswalk =
      calcSignedArcLength(ego_path.points, ego_pos, path_end_points_on_next_crosswalk->first);
    return std::min(far_attention_range, dist_to_next_crosswalk);
  }();

  const auto update_p_near =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, near_attention_range);
  const auto update_p_far =
    calcLongitudinalOffsetPoint(ego_path.points, ego_pos, far_attention_range);

  if (update_p_near && update_p_far) {
    debug_data_.range_near_point = update_p_near.value();
    debug_data_.range_far_point = update_p_far.value();
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return std::make_pair(clamped_near_attention_range, clamped_far_attention_range);
}

std::optional<double> CrosswalkModule::findEgoPassageDirectionAlongPath(
  const PathWithLaneId & path) const
{
  auto findIntersectPoint =
    [&](const lanelet::ConstLineString3d line) -> std::optional<geometry_msgs::msg::Point> {
    const auto line_start =
      autoware::universe_utils::createPoint(line.front().x(), line.front().y(), line.front().z());
    const auto line_end =
      autoware::universe_utils::createPoint(line.back().x(), line.back().y(), line.back().z());
    for (unsigned i = 0; i < path.points.size() - 1; ++i) {
      const auto & start = path.points.at(i).point.pose.position;
      const auto & end = path.points.at(i + 1).point.pose.position;
      if (const auto intersect =
            autoware::universe_utils::intersect(line_start, line_end, start, end);
          intersect.has_value()) {
        return intersect;
      }
    }
    return std::nullopt;
  };
  const auto pt1 = findIntersectPoint(crosswalk_.leftBound());
  const auto pt2 = findIntersectPoint(crosswalk_.rightBound());

  if (!pt1 || !pt2) {
    return std::nullopt;
  }

  return std::atan2(pt2->y - pt1->y, pt2->x - pt1->x);
}

std::optional<double> CrosswalkModule::findObjectPassageDirectionAlongVehicleLane(
  const autoware_perception_msgs::msg::PredictedPath & path) const
{
  using autoware::universe_utils::Segment2d;

  auto findIntersectPoint = [&](const lanelet::ConstLineString3d line)
    -> std::optional<std::pair<size_t, geometry_msgs::msg::Point>> {
    const auto line_start =
      autoware::universe_utils::createPoint(line.front().x(), line.front().y(), line.front().z());
    const auto line_end =
      autoware::universe_utils::createPoint(line.back().x(), line.back().y(), line.back().z());
    for (unsigned i = 0; i < path.path.size() - 1; ++i) {
      const auto & start = path.path.at(i).position;
      const auto & end = path.path.at(i + 1).position;
      if (const auto intersect =
            autoware::universe_utils::intersect(line_start, line_end, start, end);
          intersect.has_value()) {
        return std::make_optional(std::make_pair(i, intersect.value()));
      }
    }
    return std::nullopt;
  };
  const auto intersect_pt1 = findIntersectPoint(crosswalk_.leftBound());
  const auto intersect_pt2 = findIntersectPoint(crosswalk_.rightBound());

  if (!intersect_pt1 || !intersect_pt2) {
    return std::nullopt;
  }
  const auto idx1 = intersect_pt1.value().first, idx2 = intersect_pt2.value().first;
  const auto & front = idx1 > idx2 ? intersect_pt2.value().second : intersect_pt1.value().second;
  const auto & back = idx1 > idx2 ? intersect_pt1.value().second : intersect_pt2.value().second;
  return std::atan2(back.y - front.y, back.x - front.x);
}

std::optional<CollisionPoint> CrosswalkModule::getCollisionPoint(
  const PathWithLaneId & ego_path, const PredictedObject & object,
  const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area)
{
  stop_watch_.tic(__func__);

  const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & ego_vel = planner_data_->current_velocity->twist.linear;

  const auto obj_polygon =
    createObjectPolygon(object.shape.dimensions.x, object.shape.dimensions.y);

  double minimum_stop_dist = std::numeric_limits<double>::max();
  std::optional<CollisionPoint> nearest_collision_point{std::nullopt};
  for (const auto & obj_path : object.kinematics.predicted_paths) {
    size_t start_idx{0};
    bool is_start_idx_initialized{false};
    for (size_t i = 0; i < obj_path.path.size(); ++i) {
      // For effective computation, the point and polygon intersection is calculated first.
      const auto obj_one_step_polygon = createMultiStepPolygon(obj_path.path, obj_polygon, i, i);
      const auto one_step_intersection_polygons =
        calcOverlappingPoints(obj_one_step_polygon, attention_area);
      if (!one_step_intersection_polygons.empty()) {
        if (!is_start_idx_initialized) {
          start_idx = i;
          is_start_idx_initialized = true;
        }
        if (i != obj_path.path.size() - 1) {
          // NOTE: Even if the object path does not fully cross the ego path, the path should be
          // considered.
          continue;
        }
      }

      if (!is_start_idx_initialized) {
        continue;
      }

      // Calculate multi-step object polygon, and reset start_idx
      const size_t start_idx_with_margin = std::max(static_cast<int>(start_idx) - 1, 0);
      const size_t end_idx_with_margin = std::min(i + 1, obj_path.path.size() - 1);
      const auto object_crosswalk_passage_direction =
        findObjectPassageDirectionAlongVehicleLane(obj_path);
      const auto obj_multi_step_polygon = createMultiStepPolygon(
        obj_path.path, obj_polygon, start_idx_with_margin, end_idx_with_margin);
      is_start_idx_initialized = false;

      // Calculate intersection points between object and attention area
      const auto multi_step_intersection_polygons =
        calcOverlappingPoints(obj_multi_step_polygon, attention_area);
      if (multi_step_intersection_polygons.empty()) {
        continue;
      }

      // Calculate nearest collision point among collisions with a predicted path
      Point2d boost_intersection_center_point;
      bg::centroid(multi_step_intersection_polygons.front(), boost_intersection_center_point);
      const auto intersection_center_point = createPoint(
        boost_intersection_center_point.x(), boost_intersection_center_point.y(), ego_pos.z);

      const auto dist_ego2cp =
        calcSignedArcLength(ego_path.points, ego_pos, intersection_center_point);
      constexpr double eps = 1e-3;
      const auto dist_obj2cp =
        calcArcLength(obj_path.path) < eps
          ? 0.0
          : calcSignedArcLength(obj_path.path, size_t(0), intersection_center_point);

      if (
        dist_ego2cp < crosswalk_attention_range.first ||
        crosswalk_attention_range.second < dist_ego2cp) {
        continue;
      }

      // Calculate nearest collision point among predicted paths
      if (dist_ego2cp < minimum_stop_dist) {
        minimum_stop_dist = dist_ego2cp;
        nearest_collision_point = createCollisionPoint(
          intersection_center_point, dist_ego2cp, dist_obj2cp, ego_vel, obj_vel,
          object_crosswalk_passage_direction);
        debug_data_.obj_polygons.push_back(
          toGeometryPointVector(obj_multi_step_polygon, ego_pos.z));
      }

      break;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    logger_, planner_param_.show_processing_time, "%s : %f ms", __func__,
    stop_watch_.toc(__func__, true));

  return nearest_collision_point;
}

CollisionPoint CrosswalkModule::createCollisionPoint(
  const geometry_msgs::msg::Point & nearest_collision_point, const double dist_ego2cp,
  const double dist_obj2cp, const geometry_msgs::msg::Vector3 & ego_vel,
  const geometry_msgs::msg::Vector3 & obj_vel,
  const std::optional<double> object_crosswalk_passage_direction) const
{
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(planner_param_.min_object_velocity, estimated_velocity);

  CollisionPoint collision_point{};
  collision_point.collision_point = nearest_collision_point;
  collision_point.crosswalk_passage_direction = object_crosswalk_passage_direction;

  // The decision of whether the ego vehicle or the pedestrian goes first is determined by the logic
  // for ego_pass_first or yield; even the decision for ego_pass_later does not affect this sense.
  // Hence, here, we use the length that would be appropriate for the ego_pass_first judge.
  collision_point.time_to_collision =
    std::max(0.0, dist_ego2cp - planner_data_->vehicle_info_.min_longitudinal_offset_m) /
    std::max(ego_vel.x, planner_param_.ego_min_assumed_speed);
  collision_point.time_to_vehicle = std::max(0.0, dist_obj2cp) / velocity;

  return collision_point;
}

void CrosswalkModule::applySlowDown(
  PathWithLaneId & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const float safety_slow_down_speed)
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_path = output;
  std::optional<Pose> slowdown_pose{std::nullopt};

  if (!passed_safety_slow_point_) {
    // Safety slow down distance [m]
    const double safety_slow_down_distance =
      crosswalk_.attributeOr("safety_slow_down_distance", 2.0);

    // the range until to the point where ego will have a const safety slow down speed
    const double safety_slow_margin =
      planner_data_->vehicle_info_.max_longitudinal_offset_m + safety_slow_down_distance;
    const double safety_slow_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, first_path_point_on_crosswalk) -
      safety_slow_margin;

    const auto & p_safety_slow =
      calcLongitudinalOffsetPoint(ego_path.points, ego_pos, safety_slow_point_range);

    if (p_safety_slow.has_value()) {
      insertDecelPointWithDebugInfo(p_safety_slow.value(), safety_slow_down_speed, output);
      slowdown_pose.emplace();
      slowdown_pose->position = p_safety_slow.value();
    }

    if (safety_slow_point_range < 0.0) {
      passed_safety_slow_point_ = true;
    }
  } else {
    // the range until to the point where ego will start accelerate
    const double safety_slow_end_point_range =
      calcSignedArcLength(ego_path.points, ego_pos, last_path_point_on_crosswalk);

    if (0.0 < safety_slow_end_point_range) {
      // insert constant ego speed until the end of the crosswalk
      for (auto & p : output.points) {
        const float original_velocity = p.point.longitudinal_velocity_mps;
        p.point.longitudinal_velocity_mps = std::min(original_velocity, safety_slow_down_speed);
      }
      if (!output.points.empty()) slowdown_pose = output.points.front().point.pose;
    }
  }
  if (slowdown_pose)
    planning_factor_interface_->add(
      output.points, planner_data_->current_odometry->pose, *slowdown_pose, *slowdown_pose,
      tier4_planning_msgs::msg::PlanningFactor::SLOW_DOWN,
      tier4_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
      safety_slow_down_speed, 0.0 /*shift distance*/, "crosswalk_safety_slowdown_for_approaching");
}

void CrosswalkModule::applySlowDownByLanelet2Map(
  PathWithLaneId & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk)
{
  if (!crosswalk_.hasAttribute("safety_slow_down_speed")) {
    return;
  }
  applySlowDown(
    output, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
    static_cast<float>(crosswalk_.attribute("safety_slow_down_speed").asDouble().get()));
}

void CrosswalkModule::applySlowDownByOcclusion(
  PathWithLaneId & output, const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk)
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto objects_ptr = planner_data_->predicted_objects;

  const auto now = clock_->now();
  const auto cmp_with_time_buffer = [&](const auto & t, const auto cmp_fn) {
    return t && cmp_fn((now - *t).seconds(), planner_param_.occlusion_time_buffer);
  };
  const auto crosswalk_has_traffic_light =
    !crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>().empty();
  const auto is_crosswalk_ignored =
    (planner_param_.occlusion_ignore_with_traffic_light && crosswalk_has_traffic_light) ||
    crosswalk_.hasAttribute("skip_occluded_slowdown");
  if (!planner_param_.occlusion_enable || is_crosswalk_ignored) {
    return;
  }
  const auto dist_ego_to_crosswalk =
    calcSignedArcLength(output.points, ego_pos, first_path_point_on_crosswalk);
  const auto is_ego_on_the_crosswalk =
    dist_ego_to_crosswalk <= planner_data_->vehicle_info_.max_longitudinal_offset_m;
  if (is_ego_on_the_crosswalk) {
    return;
  }
  const auto detection_range =
    planner_data_->vehicle_info_.max_lateral_offset_m +
    calculate_detection_range(
      planner_param_.occlusion_occluded_object_velocity, dist_ego_to_crosswalk,
      planner_data_->current_velocity->twist.linear.x);
  const auto detection_areas = calculate_detection_areas(
    crosswalk_, {first_path_point_on_crosswalk.x, first_path_point_on_crosswalk.y},
    detection_range);
  debug_data_.occlusion_detection_areas = detection_areas;
  debug_data_.crosswalk_origin = first_path_point_on_crosswalk;
  if (is_crosswalk_occluded(
        *planner_data_->occupancy_grid, detection_areas, objects_ptr->objects, planner_param_)) {
    if (!current_initial_occlusion_time_) {
      current_initial_occlusion_time_ = now;
    }
    if (cmp_with_time_buffer(current_initial_occlusion_time_, std::greater_equal<double>{})) {
      most_recent_occlusion_time_ = now;
    }
  } else if (!cmp_with_time_buffer(most_recent_occlusion_time_, std::greater<double>{})) {
    current_initial_occlusion_time_.reset();
  }

  if (cmp_with_time_buffer(most_recent_occlusion_time_, std::less_equal<double>{})) {
    const auto target_velocity = calcTargetVelocity(first_path_point_on_crosswalk, output);
    applySlowDown(
      output, first_path_point_on_crosswalk, last_path_point_on_crosswalk,
      std::max(target_velocity, planner_param_.occlusion_slow_down_velocity));
    debug_data_.virtual_wall_suffix = " (occluded)";
  } else {
    most_recent_occlusion_time_.reset();
  }
}

Polygon2d CrosswalkModule::getAttentionArea(
  const PathWithLaneId & sparse_resample_path,
  const std::pair<double, double> & crosswalk_attention_range) const
{
  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto ego_polygon = createVehiclePolygon(planner_data_->vehicle_info_);
  const auto backward_path_length =
    calcSignedArcLength(sparse_resample_path.points, size_t(0), ego_pos);
  const auto length_sum = calcSignedArcLengthPartialSum(
    sparse_resample_path.points, size_t(0), sparse_resample_path.points.size());

  Polygon2d attention_area;
  for (size_t j = 0; j < sparse_resample_path.points.size() - 1; ++j) {
    const auto front_length = length_sum.at(j) - backward_path_length;
    const auto back_length = length_sum.at(j + 1) - backward_path_length;

    if (back_length < crosswalk_attention_range.first) {
      continue;
    }

    if (crosswalk_attention_range.second < front_length) {
      break;
    }

    const auto ego_one_step_polygon =
      createMultiStepPolygon(sparse_resample_path.points, ego_polygon, j, j + 1);

    debug_data_.ego_polygons.push_back(toGeometryPointVector(ego_one_step_polygon, ego_pos.z));

    std::vector<Polygon2d> unions;
    bg::union_(attention_area, ego_one_step_polygon, unions);
    if (!unions.empty()) {
      attention_area = unions.front();
      bg::correct(attention_area);
    }
  }

  return attention_area;
}

std::optional<StopFactor> CrosswalkModule::checkStopForStuckVehicles(
  const PathWithLaneId & ego_path, const std::vector<PredictedObject> & objects,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk,
  const geometry_msgs::msg::Point & last_path_point_on_crosswalk,
  const std::optional<geometry_msgs::msg::Pose> & stop_pose)
{
  const auto & p = planner_param_;

  if (!stop_pose) {
    return {};
  }

  // skip stuck vehicle checking for crosswalk which is in intersection.
  if (!p.enable_stuck_check_in_intersection) {
    std::string turn_direction = road_.attributeOr("turn_direction", "else");
    if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
      if (!road_.regulatoryElementsAs<const lanelet::TrafficLight>().empty()) {
        return {};
      }
    }
  }

  for (const auto & object : objects) {
    if (!isVehicle(object)) {
      continue;
    }

    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (p.stuck_vehicle_velocity < std::hypot(obj_vel.x, obj_vel.y)) {
      continue;
    }

    const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto lateral_offset = calcLateralOffset(ego_path.points, obj_pose.position);
    if (p.max_stuck_vehicle_lateral_offset < std::abs(lateral_offset)) {
      continue;
    }

    // check if STOP is required
    const double crosswalk_front_to_obj_rear =
      calcSignedArcLength(ego_path.points, first_path_point_on_crosswalk, obj_pose.position) -
      object.shape.dimensions.x / 2.0;
    const double crosswalk_back_to_obj_rear =
      calcSignedArcLength(ego_path.points, last_path_point_on_crosswalk, obj_pose.position) -
      object.shape.dimensions.x / 2.0;
    const double required_space_length =
      planner_data_->vehicle_info_.vehicle_length_m + planner_param_.required_clearance;

    if (crosswalk_front_to_obj_rear > 0.0 && crosswalk_back_to_obj_rear < required_space_length) {
      // If there exists at least one vehicle ahead, plan STOP considering min_acc, max_jerk and
      // min_jerk. Note that nearest search is not required because the stop pose independents to
      // the vehicles.
      const auto braking_distance = calcDecelDistWithJerkAndAccConstraints(
        planner_data_->current_velocity->twist.linear.x, 0.0,
        planner_data_->current_acceleration->accel.accel.linear.x, p.min_acc_for_stuck_vehicle,
        p.max_jerk_for_stuck_vehicle, p.min_jerk_for_stuck_vehicle);
      if (!braking_distance) {
        return {};
      }

      const auto & ego_pos = planner_data_->current_odometry->pose.position;
      const double dist_ego2stop =
        calcSignedArcLength(ego_path.points, ego_pos, stop_pose->position);
      const double feasible_dist_ego2stop = std::max(*braking_distance, dist_ego2stop);
      const double dist_to_ego =
        calcSignedArcLength(ego_path.points, ego_path.points.front().point.pose.position, ego_pos);
      const auto feasible_stop_pose =
        calcLongitudinalOffsetPose(ego_path.points, 0, dist_to_ego + feasible_dist_ego2stop);
      if (!feasible_stop_pose) {
        return {};
      }

      setObjectsOfInterestData(obj_pose, object.shape, ColorName::RED);
      return createStopFactor(*feasible_stop_pose, {obj_pose.position});
    }
  }

  return {};
}

std::optional<StopFactor> CrosswalkModule::getNearestStopFactor(
  const PathWithLaneId & ego_path,
  const std::optional<StopFactor> & stop_factor_for_crosswalk_users,
  const std::optional<StopFactor> & stop_factor_for_stuck_vehicles)
{
  const auto get_distance_to_stop = [&](const auto stop_factor) -> std::optional<double> {
    const auto & ego_pos = planner_data_->current_odometry->pose.position;
    if (!stop_factor) return std::nullopt;
    return calcSignedArcLength(ego_path.points, ego_pos, stop_factor->stop_pose.position);
  };
  const auto dist_to_stop_for_crosswalk_users =
    get_distance_to_stop(stop_factor_for_crosswalk_users);
  const auto dist_to_stop_for_stuck_vehicles = get_distance_to_stop(stop_factor_for_stuck_vehicles);

  if (dist_to_stop_for_crosswalk_users) {
    if (dist_to_stop_for_stuck_vehicles) {
      if (*dist_to_stop_for_stuck_vehicles < *dist_to_stop_for_crosswalk_users) {
        return stop_factor_for_stuck_vehicles;
      }
    }
    return stop_factor_for_crosswalk_users;
  }

  if (dist_to_stop_for_stuck_vehicles) {
    return stop_factor_for_stuck_vehicles;
  }

  return {};
}

void CrosswalkModule::updateObjectState(
  const double dist_ego_to_stop, const PathWithLaneId & sparse_resample_path,
  const std::pair<double, double> & crosswalk_attention_range, const Polygon2d & attention_area)
{
  const auto & objects_ptr = planner_data_->predicted_objects;

  const auto traffic_lights_reg_elems =
    crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>();
  const bool has_traffic_light = !traffic_lights_reg_elems.empty();

  // Check if ego is yielding
  const bool is_ego_yielding = [&]() {
    const auto has_reached_stop_point = dist_ego_to_stop < planner_param_.stop_position_threshold;

    return planner_data_->isVehicleStopped(planner_param_.timeout_ego_stop_for_yield) &&
           has_reached_stop_point;
  }();

  const auto ignore_crosswalk = isRedSignalForPedestrians();
  debug_data_.ignore_crosswalk = ignore_crosswalk;

  // Update object state
  object_info_manager_.init();
  for (const auto & object : objects_ptr->objects) {
    const auto obj_uuid = toHexString(object.object_id);
    const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    const auto & obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;

    // calculate collision point and state
    if (!isCrosswalkUserType(object)) {
      continue;
    }
    if (ignore_crosswalk) {
      continue;
    }

    const auto collision_point =
      getCollisionPoint(sparse_resample_path, object, crosswalk_attention_range, attention_area);
    const std::optional<double> ego_crosswalk_passage_direction =
      findEgoPassageDirectionAlongPath(sparse_resample_path);
    object_info_manager_.update(
      obj_uuid, obj_pos, std::hypot(obj_vel.x, obj_vel.y), clock_->now(), is_ego_yielding,
      has_traffic_light, collision_point, object.classification.front().label, planner_param_,
      crosswalk_.polygon2d().basicPolygon(), attention_area, ego_crosswalk_passage_direction);

    const auto collision_state = object_info_manager_.getCollisionState(obj_uuid);
    if (collision_point) {
      debug_data_.collision_points.push_back(
        std::make_tuple(obj_uuid, *collision_point, collision_state));
    }

    const auto getLabelColor = [](const auto collision_state) {
      if (collision_state == CollisionState::YIELD) {
        return ColorName::RED;
      } else if (
        collision_state == CollisionState::EGO_PASS_FIRST ||
        collision_state == CollisionState::EGO_PASS_LATER) {
        return ColorName::GREEN;
      } else if (collision_state == CollisionState::IGNORE) {
        return ColorName::GRAY;
      } else {
        return ColorName::AMBER;
      }
    };

    setObjectsOfInterestData(
      object.kinematics.initial_pose_with_covariance.pose, object.shape,
      getLabelColor(collision_state));
  }

  object_info_manager_.finalize();
}

bool CrosswalkModule::isRedSignalForPedestrians() const
{
  const auto traffic_lights_reg_elems =
    crosswalk_.regulatoryElementsAs<const lanelet::TrafficLight>();

  for (const auto & traffic_lights_reg_elem : traffic_lights_reg_elems) {
    const auto traffic_signal_stamped_opt =
      planner_data_->getTrafficSignal(traffic_lights_reg_elem->id());
    if (!traffic_signal_stamped_opt) {
      continue;
    }
    const auto traffic_signal_stamped = traffic_signal_stamped_opt.value();

    if (
      planner_param_.traffic_light_state_timeout <
      (clock_->now() - traffic_signal_stamped.stamp).seconds()) {
      continue;
    }

    const auto & lights = traffic_signal_stamped.signal.elements;
    if (lights.empty()) {
      continue;
    }

    for (const auto & element : lights) {
      if (element.color == TrafficLightElement::RED && element.shape == TrafficLightElement::CIRCLE)
        return true;
    }
  }

  return false;
}

bool CrosswalkModule::isVehicle(const PredictedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::CAR) {
    return true;
  }

  if (label == ObjectClassification::TRUCK) {
    return true;
  }

  if (label == ObjectClassification::BUS) {
    return true;
  }

  if (label == ObjectClassification::TRAILER) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE) {
    return true;
  }

  return false;
}

bool CrosswalkModule::isCrosswalkUserType(const PredictedObject & object) const
{
  if (object.classification.empty()) {
    return false;
  }

  const auto & label = object.classification.front().label;

  if (label == ObjectClassification::UNKNOWN && planner_param_.look_unknown) {
    return true;
  }

  if (label == ObjectClassification::BICYCLE && planner_param_.look_bicycle) {
    return true;
  }

  if (label == ObjectClassification::MOTORCYCLE && planner_param_.look_motorcycle) {
    return true;
  }

  if (label == ObjectClassification::PEDESTRIAN && planner_param_.look_pedestrian) {
    return true;
  }

  return false;
}

geometry_msgs::msg::Polygon CrosswalkModule::createObjectPolygon(
  const double width_m, const double length_m)
{
  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m / 2.0, -width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, width_m / 2.0, 0.0));
  polygon.points.push_back(createPoint32(-length_m / 2.0, -width_m / 2.0, 0.0));

  return polygon;
}

geometry_msgs::msg::Polygon CrosswalkModule::createVehiclePolygon(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0;
  const auto & back_m = i.rear_overhang_m;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(front_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(front_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-back_m, -width_m, 0.0));

  return polygon;
}

void CrosswalkModule::setDistanceToStop(
  const PathWithLaneId & ego_path,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose,
  const std::optional<StopFactor> & stop_factor)
{
  // calculate stop position
  const auto stop_pos = [&]() -> std::optional<geometry_msgs::msg::Point> {
    if (stop_factor) return stop_factor->stop_pose.position;
    if (default_stop_pose) return default_stop_pose->position;
    return std::nullopt;
  }();

  // Set distance
  if (stop_pos) {
    const auto & ego_pos = planner_data_->current_odometry->pose.position;
    const double dist_ego2stop = calcSignedArcLength(ego_path.points, ego_pos, *stop_pos);
    setDistance(dist_ego2stop);
  } else {
    setDistance(std::numeric_limits<double>::lowest());
  }
}

void CrosswalkModule::planGo(
  PathWithLaneId & ego_path, const std::optional<StopFactor> & stop_factor) const
{
  if (!stop_factor.has_value()) {
    return;
  }
  // Plan slow down
  const auto target_velocity = calcTargetVelocity(stop_factor->stop_pose.position, ego_path);
  insertDecelPointWithDebugInfo(
    stop_factor->stop_pose.position,
    std::max(planner_param_.min_slow_down_velocity, target_velocity), ego_path);
}

void CrosswalkModule::planStop(
  PathWithLaneId & ego_path, const std::optional<StopFactor> & nearest_stop_factor,
  const std::optional<geometry_msgs::msg::Pose> & default_stop_pose)
{
  // Calculate stop factor
  auto stop_factor = [&]() -> std::optional<StopFactor> {
    if (nearest_stop_factor) return *nearest_stop_factor;
    if (default_stop_pose) return createStopFactor(*default_stop_pose);
    return std::nullopt;
  }();

  if (!stop_factor) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 5000, "stop_factor is null");
    return;
  }

  // Check if the restart should be suppressed.
  const bool suppress_restart = checkRestartSuppression(ego_path, stop_factor);
  if (suppress_restart) {
    const auto & ego_pose = planner_data_->current_odometry->pose;
    stop_factor->stop_pose = ego_pose;
  }

  // Plan stop
  insertDecelPointWithDebugInfo(stop_factor->stop_pose.position, 0.0, ego_path);
  planning_factor_interface_->add(
    ego_path.points, planner_data_->current_odometry->pose, stop_factor->stop_pose,
    stop_factor->stop_pose, tier4_planning_msgs::msg::PlanningFactor::STOP,
    tier4_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0 /*velocity*/,
    0.0 /*shift distance*/, "crosswalk_stop");
}

bool CrosswalkModule::checkRestartSuppression(
  const PathWithLaneId & ego_path, const std::optional<StopFactor> & stop_factor) const
{
  if (!planner_data_->isVehicleStopped()) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const double dist_to_stop =
    calcSignedArcLength(ego_path.points, ego_pos, stop_factor->stop_pose.position);

  // NOTE: min_dist_to_stop_for_restart_suppression is supposed to be the same as
  //      the pid_longitudinal_controller's drive_state_stop_dist.
  return planner_param_.min_dist_to_stop_for_restart_suppression < dist_to_stop &&
         dist_to_stop < planner_param_.max_dist_to_stop_for_restart_suppression;
}
}  // namespace autoware::behavior_velocity_planner
