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

#include "autoware/obstacle_cruise_planner/planner_interface.hpp"

#include "autoware/motion_utils/distance/distance.hpp"
#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/signal_processing/lowpass_filter_1d.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <algorithm>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
StopSpeedExceeded createStopSpeedExceededMsg(
  const rclcpp::Time & current_time, const bool stop_flag)
{
  StopSpeedExceeded msg{};
  msg.stamp = current_time;
  msg.stop_speed_exceeded = stop_flag;
  return msg;
}

double calcMinimumDistanceToStop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

template <typename T>
std::optional<T> getObjectFromUuid(const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

// TODO(murooka) following two functions are copied from behavior_velocity_planner.
// These should be refactored.
double findReachTime(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    throw std::logic_error("[obstacle_cruise_planner](findReachTime): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    // std::cout<<"fx: "<<fx<<" up: "<<upper<<" lo: "<<lower<<" t: "<<t<<std::endl;
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[obstacle_cruise_planner](findReachTime): current iter is over warning"
                << std::endl;
  }
  return t;
}

double calcDecelerationVelocityFromDistanceToTarget(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    throw std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = findReachTime(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

autoware::universe_utils::Point2d convertPoint(const geometry_msgs::msg::Point & p)
{
  return autoware::universe_utils::Point2d{p.x, p.y};
}
}  // namespace

std::vector<TrajectoryPoint> PlannerInterface::generateStopTrajectory(
  const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles)
{
  stop_watch_.tic(__func__);

  stop_planning_debug_info_.reset();
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_vel);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_acc);

  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  if (stop_obstacles.empty()) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return planner_data.traj_points;
  }

  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), enable_debug_info_,
    "stop planning");

  std::optional<StopObstacle> determined_stop_obstacle{};
  std::optional<double> determined_zero_vel_dist{};
  std::optional<double> determined_desired_margin{};

  const auto closest_stop_obstacles =
    obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles);
  for (const auto & stop_obstacle : closest_stop_obstacles) {
    const auto ego_segment_idx =
      ego_nearest_param_.findSegmentIndex(planner_data.traj_points, planner_data.ego_pose);
    const double dist_to_collide_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(planner_data.traj_points, 0, ego_segment_idx) +
      stop_obstacle.dist_to_collide_on_decimated_traj;

    const double desired_margin = [&]() {
      const double margin_from_obstacle =
        calculateMarginFromObstacleOnCurve(planner_data, stop_obstacle);
      // Use terminal margin (terminal_safe_distance_margin) for obstacle stop
      const auto ref_traj_length = autoware::motion_utils::calcSignedArcLength(
        planner_data.traj_points, 0, planner_data.traj_points.size() - 1);
      if (dist_to_collide_on_ref_traj > ref_traj_length) {
        return longitudinal_info_.terminal_safe_distance_margin;
      }

      // If behavior stop point is ahead of the closest_obstacle_stop point within a certain
      // margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
      const auto closest_behavior_stop_idx = autoware::motion_utils::searchZeroVelocityIndex(
        planner_data.traj_points, ego_segment_idx + 1);
      if (closest_behavior_stop_idx) {
        const double closest_behavior_stop_dist_on_ref_traj =
          autoware::motion_utils::calcSignedArcLength(
            planner_data.traj_points, 0, *closest_behavior_stop_idx);
        const double stop_dist_diff = closest_behavior_stop_dist_on_ref_traj -
                                      (dist_to_collide_on_ref_traj - margin_from_obstacle);
        if (0.0 < stop_dist_diff && stop_dist_diff < margin_from_obstacle) {
          return min_behavior_stop_margin_;
        }
      }
      return margin_from_obstacle;
    }();

    // calc stop point against the obstacle
    double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_margin);
    if (suppress_sudden_obstacle_stop_) {
      const auto acceptable_stop_acc = [&]() -> std::optional<double> {
        if (stop_param_.getParamType(stop_obstacle.classification) == "default") {
          return longitudinal_info_.limit_min_accel;
        }
        const double distance_to_judge_suddenness = std::min(
          calcMinimumDistanceToStop(
            planner_data.ego_vel, longitudinal_info_.limit_max_accel,
            stop_param_.getParam(stop_obstacle.classification).sudden_object_acc_threshold),
          stop_param_.getParam(stop_obstacle.classification).sudden_object_dist_threshold);
        if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
          return longitudinal_info_.limit_min_accel;
        }
        if (stop_param_.getParam(stop_obstacle.classification).abandon_to_stop) {
          RCLCPP_WARN(
            rclcpp::get_logger("ObstacleCruisePlanner::StopPlanner"),
            "[Cruise] abandon to stop against %s object",
            stop_param_.types_maps.at(stop_obstacle.classification.label).c_str());
          return std::nullopt;
        } else {
          return stop_param_.getParam(stop_obstacle.classification).limit_min_acc;
        }
      }();
      if (!acceptable_stop_acc) {
        continue;
      }

      const double acceptable_stop_pos =
        autoware::motion_utils::calcSignedArcLength(
          planner_data.traj_points, 0, planner_data.ego_pose.position) +
        calcMinimumDistanceToStop(
          planner_data.ego_vel, longitudinal_info_.limit_max_accel, acceptable_stop_acc.value());
      if (acceptable_stop_pos > candidate_zero_vel_dist) {
        candidate_zero_vel_dist = acceptable_stop_pos;
      }
    }

    if (determined_stop_obstacle) {
      const bool is_same_param_types =
        (stop_obstacle.classification.label == determined_stop_obstacle->classification.label);
      if (
        (is_same_param_types && stop_obstacle.dist_to_collide_on_decimated_traj >
                                  determined_stop_obstacle->dist_to_collide_on_decimated_traj) ||
        (!is_same_param_types && candidate_zero_vel_dist > determined_zero_vel_dist)) {
        continue;
      }
    }
    determined_zero_vel_dist = candidate_zero_vel_dist;
    determined_stop_obstacle = stop_obstacle;
    determined_desired_margin = desired_margin;
  }

  if (!determined_zero_vel_dist) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return planner_data.traj_points;
  }

  // Hold previous stop distance if necessary
  if (
    std::abs(planner_data.ego_vel) < longitudinal_info_.hold_stop_velocity_threshold &&
    prev_stop_distance_info_) {
    // NOTE: We assume that the current trajectory's front point is ahead of the previous
    // trajectory's front point.
    const size_t traj_front_point_prev_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_stop_distance_info_->first, planner_data.traj_points.front().pose);
    const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
      prev_stop_distance_info_->first, 0, planner_data.traj_points.front().pose.position,
      traj_front_point_prev_seg_idx);

    const double prev_zero_vel_dist = prev_stop_distance_info_->second - diff_dist_front_points;
    if (
      std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
      longitudinal_info_.hold_stop_distance_threshold) {
      determined_zero_vel_dist.value() = prev_zero_vel_dist;
    }
  }

  // Insert stop point
  auto output_traj_points = planner_data.traj_points;
  const auto zero_vel_idx =
    autoware::motion_utils::insertStopPoint(0, *determined_zero_vel_dist, output_traj_points);
  if (zero_vel_idx) {
    // virtual wall marker for stop obstacle
    const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
      output_traj_points.at(*zero_vel_idx).pose, "obstacle stop", planner_data.current_time, 0,
      abs_ego_offset, "", planner_data.is_driving_forward);
    autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);
    debug_data_ptr_->obstacles_to_stop.push_back(*determined_stop_obstacle);

    // Publish Stop Reason
    const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
    planning_factor_interface_->add(
      output_traj_points, planner_data.ego_pose, stop_pose,
      tier4_planning_msgs::msg::PlanningFactor::STOP,
      tier4_planning_msgs::msg::SafetyFactorArray{});
    // Store stop reason debug data
    debug_data_ptr_->stop_metrics =
      makeMetrics("PlannerInterface", "stop", planner_data, stop_pose, *determined_stop_obstacle);
    // Publish if ego vehicle will over run against the stop point with a limit acceleration

    const bool will_over_run = determined_zero_vel_dist.value() >
                               autoware::motion_utils::calcSignedArcLength(
                                 planner_data.traj_points, 0, planner_data.ego_pose.position) +
                                 determined_stop_obstacle->dist_to_collide_on_decimated_traj +
                                 determined_desired_margin.value() + 0.1;
    const auto stop_speed_exceeded_msg =
      createStopSpeedExceededMsg(planner_data.current_time, will_over_run);
    stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);

    prev_stop_distance_info_ = std::make_pair(output_traj_points, determined_zero_vel_dist.value());
  }

  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_DISTANCE,
    determined_stop_obstacle->dist_to_collide_on_decimated_traj);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_VELOCITY,
    determined_stop_obstacle->velocity);
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_TARGET_OBSTACLE_DISTANCE, determined_desired_margin.value());
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_VELOCITY, 0.0);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_ACCELERATION, 0.0);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return output_traj_points;
}

double PlannerInterface::calculateMarginFromObstacleOnCurve(
  const PlannerData & planner_data, const StopObstacle & stop_obstacle) const
{
  if (!enable_approaching_on_curve_ || use_pointcloud_) {
    return longitudinal_info_.safe_distance_margin;
  }

  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  // calculate short trajectory points towards obstacle
  const size_t obj_segment_idx = autoware::motion_utils::findNearestSegmentIndex(
    planner_data.traj_points, stop_obstacle.collision_point);
  std::vector<TrajectoryPoint> short_traj_points{planner_data.traj_points.at(obj_segment_idx + 1)};
  double sum_short_traj_length{0.0};
  for (int i = obj_segment_idx; 0 <= i; --i) {
    short_traj_points.push_back(planner_data.traj_points.at(i));

    if (
      1 < short_traj_points.size() &&
      longitudinal_info_.safe_distance_margin + abs_ego_offset < sum_short_traj_length) {
      break;
    }
    sum_short_traj_length += autoware::universe_utils::calcDistance2d(
      planner_data.traj_points.at(i), planner_data.traj_points.at(i + 1));
  }
  std::reverse(short_traj_points.begin(), short_traj_points.end());
  if (short_traj_points.size() < 2) {
    return longitudinal_info_.safe_distance_margin;
  }

  // calculate collision index between straight line from ego pose and object
  const auto calculate_distance_from_straight_ego_path =
    [&](const auto & ego_pose, const auto & object_polygon) {
      const auto forward_ego_pose = autoware::universe_utils::calcOffsetPose(
        ego_pose, longitudinal_info_.safe_distance_margin + 3.0, 0.0, 0.0);
      const auto ego_straight_segment = autoware::universe_utils::Segment2d{
        convertPoint(ego_pose.position), convertPoint(forward_ego_pose.position)};
      return boost::geometry::distance(ego_straight_segment, object_polygon);
    };
  const auto resampled_short_traj_points = resampleTrajectoryPoints(short_traj_points, 0.5);
  const auto object_polygon =
    autoware::universe_utils::toPolygon2d(stop_obstacle.pose, stop_obstacle.shape);
  const auto collision_idx = [&]() -> std::optional<size_t> {
    for (size_t i = 0; i < resampled_short_traj_points.size(); ++i) {
      const double dist_to_obj = calculate_distance_from_straight_ego_path(
        resampled_short_traj_points.at(i).pose, object_polygon);
      if (dist_to_obj < vehicle_info_.vehicle_width_m / 2.0) {
        return i;
      }
    }
    return std::nullopt;
  }();
  if (!collision_idx) {
    return min_safe_distance_margin_on_curve_;
  }
  if (*collision_idx == 0) {
    return longitudinal_info_.safe_distance_margin;
  }

  // calculate margin from obstacle
  const double partial_segment_length = [&]() {
    const double collision_segment_length = autoware::universe_utils::calcDistance2d(
      resampled_short_traj_points.at(*collision_idx - 1),
      resampled_short_traj_points.at(*collision_idx));
    const double prev_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx - 1).pose, object_polygon);
    const double next_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx).pose, object_polygon);
    return (next_dist - vehicle_info_.vehicle_width_m / 2.0) / (next_dist - prev_dist) *
           collision_segment_length;
  }();

  const double short_margin_from_obstacle =
    partial_segment_length +
    autoware::motion_utils::calcSignedArcLength(
      resampled_short_traj_points, *collision_idx, stop_obstacle.collision_point) -
    abs_ego_offset + additional_safe_distance_margin_on_curve_;

  return std::min(
    longitudinal_info_.safe_distance_margin,
    std::max(min_safe_distance_margin_on_curve_, short_margin_from_obstacle));
}

double PlannerInterface::calcDistanceToCollisionPoint(
  const PlannerData & planner_data, const geometry_msgs::msg::Point & collision_point)
{
  const double offset = planner_data.is_driving_forward
                          ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                          : std::abs(vehicle_info_.min_longitudinal_offset_m);

  const size_t ego_segment_idx =
    ego_nearest_param_.findSegmentIndex(planner_data.traj_points, planner_data.ego_pose);

  const size_t collision_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(planner_data.traj_points, collision_point);

  const auto dist_to_collision_point = autoware::motion_utils::calcSignedArcLength(
    planner_data.traj_points, planner_data.ego_pose.position, ego_segment_idx, collision_point,
    collision_segment_idx);

  return dist_to_collision_point - offset;
}

std::vector<TrajectoryPoint> PlannerInterface::generateSlowDownTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & cruise_traj_points,
  const std::vector<SlowDownObstacle> & obstacles,
  [[maybe_unused]] std::optional<VelocityLimit> & vel_limit)
{
  stop_watch_.tic(__func__);
  auto slow_down_traj_points = cruise_traj_points;
  slow_down_debug_multi_array_ = Float32MultiArrayStamped();

  const double dist_to_ego = [&]() {
    const size_t ego_seg_idx =
      ego_nearest_param_.findSegmentIndex(slow_down_traj_points, planner_data.ego_pose);
    return autoware::motion_utils::calcSignedArcLength(
      slow_down_traj_points, 0, planner_data.ego_pose.position, ego_seg_idx);
  }();
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  // define function to insert slow down velocity to trajectory
  const auto insert_point_in_trajectory = [&](const double lon_dist) -> std::optional<size_t> {
    const auto inserted_idx =
      autoware::motion_utils::insertTargetPoint(0, lon_dist, slow_down_traj_points);
    if (inserted_idx) {
      if (inserted_idx.value() + 1 <= slow_down_traj_points.size() - 1) {
        // zero-order hold for velocity interpolation
        slow_down_traj_points.at(inserted_idx.value()).longitudinal_velocity_mps =
          slow_down_traj_points.at(inserted_idx.value() + 1).longitudinal_velocity_mps;
      }
      return inserted_idx.value();
    }
    return std::nullopt;
  };

  std::vector<SlowDownOutput> new_prev_slow_down_output;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obstacle = obstacles.at(i);
    const auto prev_output = getObjectFromUuid(prev_slow_down_output_, obstacle.uuid);

    const bool is_obstacle_moving = [&]() -> bool {
      const auto object_vel_norm = std::hypot(obstacle.velocity, obstacle.lat_velocity);
      if (!prev_output) return object_vel_norm > moving_object_speed_threshold;
      if (prev_output->is_moving)
        return object_vel_norm > moving_object_speed_threshold - moving_object_hysteresis_range;
      return object_vel_norm > moving_object_speed_threshold + moving_object_hysteresis_range;
    }();

    // calculate slow down start distance, and insert slow down velocity
    const auto dist_vec_to_slow_down = calculateDistanceToSlowDownWithConstraints(
      planner_data, slow_down_traj_points, obstacle, prev_output, dist_to_ego, is_obstacle_moving);
    if (!dist_vec_to_slow_down) {
      RCLCPP_INFO_EXPRESSION(
        rclcpp::get_logger("ObstacleCruisePlanner::PlannerInterface"), enable_debug_info_,
        "[SlowDown] Ignore obstacle (%s) since distance to slow down is not valid",
        obstacle.uuid.c_str());
      continue;
    }
    const auto dist_to_slow_down_start = std::get<0>(*dist_vec_to_slow_down);
    const auto dist_to_slow_down_end = std::get<1>(*dist_vec_to_slow_down);
    const auto feasible_slow_down_vel = std::get<2>(*dist_vec_to_slow_down);

    // calculate slow down end distance, and insert slow down velocity
    // NOTE: slow_down_start_idx will not be wrong since inserted back point is after inserted
    // front point.
    const auto slow_down_start_idx = insert_point_in_trajectory(dist_to_slow_down_start);
    const auto slow_down_end_idx = dist_to_slow_down_start < dist_to_slow_down_end
                                     ? insert_point_in_trajectory(dist_to_slow_down_end)
                                     : std::nullopt;
    if (!slow_down_end_idx) {
      continue;
    }

    // calculate slow down velocity
    const double stable_slow_down_vel = [&]() {
      if (prev_output) {
        return autoware::signal_processing::lowpassFilter(
          feasible_slow_down_vel, prev_output->target_vel, slow_down_param_.lpf_gain_slow_down_vel);
      }
      return feasible_slow_down_vel;
    }();

    // insert slow down velocity between slow start and end
    for (size_t j = (slow_down_start_idx ? *slow_down_start_idx : 0); j <= *slow_down_end_idx;
         ++j) {
      auto & traj_point = slow_down_traj_points.at(j);
      traj_point.longitudinal_velocity_mps =
        std::min(traj_point.longitudinal_velocity_mps, static_cast<float>(stable_slow_down_vel));
    }

    // add debug data
    slow_down_debug_multi_array_.data.push_back(obstacle.precise_lat_dist);
    slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_start);
    slow_down_debug_multi_array_.data.push_back(dist_to_slow_down_end);
    slow_down_debug_multi_array_.data.push_back(feasible_slow_down_vel);
    slow_down_debug_multi_array_.data.push_back(stable_slow_down_vel);
    slow_down_debug_multi_array_.data.push_back(slow_down_start_idx ? *slow_down_start_idx : -1.0);
    slow_down_debug_multi_array_.data.push_back(*slow_down_end_idx);

    // add virtual wall
    if (slow_down_start_idx && slow_down_end_idx) {
      const size_t ego_idx =
        ego_nearest_param_.findIndex(slow_down_traj_points, planner_data.ego_pose);
      const size_t slow_down_wall_idx = [&]() {
        if (ego_idx < *slow_down_start_idx) return *slow_down_start_idx;
        if (ego_idx < *slow_down_end_idx) return ego_idx;
        return *slow_down_end_idx;
      }();

      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(slow_down_wall_idx).pose, "obstacle slow down",
        planner_data.current_time, i, abs_ego_offset, "", planner_data.is_driving_forward);
      autoware::universe_utils::appendMarkerArray(markers, &debug_data_ptr_->slow_down_wall_marker);
      planning_factor_interface_->add(
        slow_down_traj_points, planner_data.ego_pose,
        slow_down_traj_points.at(*slow_down_start_idx).pose,
        slow_down_traj_points.at(*slow_down_end_idx).pose,
        tier4_planning_msgs::msg::PlanningFactor::SLOW_DOWN,
        tier4_planning_msgs::msg::SafetyFactorArray{}, planner_data.is_driving_forward,
        stable_slow_down_vel);
    }

    // add debug virtual wall
    if (slow_down_start_idx) {
      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(*slow_down_start_idx).pose, "obstacle slow down start",
        planner_data.current_time, i * 2, abs_ego_offset, "", planner_data.is_driving_forward);
      autoware::universe_utils::appendMarkerArray(
        markers, &debug_data_ptr_->slow_down_debug_wall_marker);
    }
    if (slow_down_end_idx) {
      const auto markers = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(*slow_down_end_idx).pose, "obstacle slow down end",
        planner_data.current_time, i * 2 + 1, abs_ego_offset, "", planner_data.is_driving_forward);
      autoware::universe_utils::appendMarkerArray(
        markers, &debug_data_ptr_->slow_down_debug_wall_marker);
    }

    // Add debug data
    debug_data_ptr_->obstacles_to_slow_down.push_back(obstacle);
    if (!debug_data_ptr_->stop_metrics.has_value()) {
      debug_data_ptr_->slow_down_metrics =
        makeMetrics("PlannerInterface", "slow_down", planner_data);
    }

    // update prev_slow_down_output_
    new_prev_slow_down_output.push_back(SlowDownOutput{
      obstacle.uuid, slow_down_traj_points, slow_down_start_idx, slow_down_end_idx,
      stable_slow_down_vel, feasible_slow_down_vel, obstacle.precise_lat_dist, is_obstacle_moving});
  }

  // update prev_slow_down_output_
  prev_slow_down_output_ = new_prev_slow_down_output;

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return slow_down_traj_points;
}

double PlannerInterface::calculateSlowDownVelocity(
  const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
  const bool is_obstacle_moving) const
{
  const auto & p =
    slow_down_param_.getObstacleParamByLabel(obstacle.classification, is_obstacle_moving);
  const double stable_precise_lat_dist = [&]() {
    if (prev_output) {
      return autoware::signal_processing::lowpassFilter(
        obstacle.precise_lat_dist, prev_output->precise_lat_dist,
        slow_down_param_.lpf_gain_lat_dist);
    }
    return obstacle.precise_lat_dist;
  }();

  const double ratio = std::clamp(
    (std::abs(stable_precise_lat_dist) - p.min_lat_margin) / (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  const double slow_down_vel =
    p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

  return slow_down_vel;
}

std::optional<std::tuple<double, double, double>>
PlannerInterface::calculateDistanceToSlowDownWithConstraints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
  const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
  const double dist_to_ego, const bool is_obstacle_moving) const
{
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);
  const double obstacle_vel = obstacle.velocity;
  // calculate slow down velocity
  const double slow_down_vel = calculateSlowDownVelocity(obstacle, prev_output, is_obstacle_moving);

  // calculate distance to collision points
  const double dist_to_front_collision =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.front_collision_point);
  const double dist_to_back_collision =
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, obstacle.back_collision_point);

  // calculate offset distance to first collision considering relative velocity
  const double relative_vel = planner_data.ego_vel - obstacle.velocity;
  const double offset_dist_to_collision = [&]() {
    if (dist_to_front_collision < dist_to_ego + abs_ego_offset) {
      return 0.0;
    }

    // NOTE: This min_relative_vel forces the relative velocity positive if the ego velocity is
    // lower than the obstacle velocity. Without this, the slow down feature will flicker where
    // the ego velocity is very close to the obstacle velocity.
    constexpr double min_relative_vel = 1.0;
    const double time_to_collision = (dist_to_front_collision - dist_to_ego - abs_ego_offset) /
                                     std::max(min_relative_vel, relative_vel);

    constexpr double time_to_collision_margin = 1.0;
    const double cropped_time_to_collision =
      std::max(0.0, time_to_collision - time_to_collision_margin);
    return obstacle_vel * cropped_time_to_collision;
  }();

  // calculate distance during deceleration, slow down preparation, and slow down
  const double min_slow_down_prepare_dist = 3.0;
  const double slow_down_prepare_dist = std::max(
    min_slow_down_prepare_dist, slow_down_vel * slow_down_param_.time_margin_on_target_velocity);
  const double deceleration_dist = offset_dist_to_collision + dist_to_front_collision -
                                   abs_ego_offset - dist_to_ego - slow_down_prepare_dist;
  const double slow_down_dist =
    dist_to_back_collision - dist_to_front_collision + slow_down_prepare_dist;

  // calculate distance to start/end slow down
  const double dist_to_slow_down_start = dist_to_ego + deceleration_dist;
  const double dist_to_slow_down_end = dist_to_ego + deceleration_dist + slow_down_dist;
  if (100.0 < dist_to_slow_down_start) {
    // NOTE: distance to slow down is too far.
    return std::nullopt;
  }

  // apply low-pass filter to distance to start/end slow down
  const auto apply_lowpass_filter = [&](const double dist_to_slow_down, const auto prev_point) {
    if (prev_output && prev_point) {
      const size_t seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(traj_points, prev_point->position);
      const double prev_dist_to_slow_down =
        autoware::motion_utils::calcSignedArcLength(traj_points, 0, prev_point->position, seg_idx);
      return autoware::signal_processing::lowpassFilter(
        dist_to_slow_down, prev_dist_to_slow_down, slow_down_param_.lpf_gain_dist_to_slow_down);
    }
    return dist_to_slow_down;
  };
  const double filtered_dist_to_slow_down_start =
    apply_lowpass_filter(dist_to_slow_down_start, prev_output->start_point);
  const double filtered_dist_to_slow_down_end =
    apply_lowpass_filter(dist_to_slow_down_end, prev_output->end_point);

  // calculate velocity considering constraints
  const double feasible_slow_down_vel = [&]() {
    if (deceleration_dist < 0) {
      if (prev_output) {
        return prev_output->target_vel;
      }
      return std::max(planner_data.ego_vel, slow_down_vel);
    }
    if (planner_data.ego_vel < slow_down_vel) {
      return slow_down_vel;
    }

    const double one_shot_feasible_slow_down_vel = [&]() {
      if (planner_data.ego_acc < longitudinal_info_.min_accel) {
        const double squared_vel =
          std::pow(planner_data.ego_vel, 2) + 2 * deceleration_dist * longitudinal_info_.min_accel;
        if (squared_vel < 0) {
          return slow_down_vel;
        }
        return std::max(std::sqrt(squared_vel), slow_down_vel);
      }
      // TODO(murooka) Calculate more precisely. Final acceleration should be zero.
      const double min_feasible_slow_down_vel = calcDecelerationVelocityFromDistanceToTarget(
        longitudinal_info_.slow_down_min_jerk, longitudinal_info_.slow_down_min_accel,
        planner_data.ego_acc, planner_data.ego_vel, deceleration_dist);
      return min_feasible_slow_down_vel;
    }();
    if (prev_output) {
      // NOTE: If longitudinal controllability is not good, one_shot_slow_down_vel may be getting
      // larger since we use actual ego's velocity and acceleration for its calculation.
      //       Suppress one_shot_slow_down_vel getting larger here.
      const double feasible_slow_down_vel =
        std::min(one_shot_feasible_slow_down_vel, prev_output->feasible_target_vel);
      return std::max(slow_down_vel, feasible_slow_down_vel);
    }
    return std::max(slow_down_vel, one_shot_feasible_slow_down_vel);
  }();

  return std::make_tuple(
    filtered_dist_to_slow_down_start, filtered_dist_to_slow_down_end, feasible_slow_down_vel);
}

std::vector<Metric> PlannerInterface::makeMetrics(
  const std::string & module_name, const std::string & reason,
  const std::optional<PlannerData> & planner_data,
  const std::optional<geometry_msgs::msg::Pose> & stop_pose,
  const std::optional<StopObstacle> & stop_obstacle)
{
  auto metrics = std::vector<Metric>();

  // Create status
  {
    // Decision
    Metric decision_metric;
    decision_metric.name = module_name + "/decision";
    decision_metric.unit = "string";
    decision_metric.value = reason;
    metrics.push_back(decision_metric);
  }

  if (stop_pose.has_value() && planner_data.has_value()) {  // Stop info
    Metric stop_position_metric;
    stop_position_metric.name = module_name + "/stop_position";
    stop_position_metric.unit = "string";
    const auto & p = stop_pose.value().position;
    stop_position_metric.value =
      "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
    metrics.push_back(stop_position_metric);

    Metric stop_orientation_metric;
    stop_orientation_metric.name = module_name + "/stop_orientation";
    stop_orientation_metric.unit = "string";
    const auto & o = stop_pose.value().orientation;
    stop_orientation_metric.value = "{" + std::to_string(o.w) + ", " + std::to_string(o.x) + ", " +
                                    std::to_string(o.y) + ", " + std::to_string(o.z) + "}";
    metrics.push_back(stop_orientation_metric);

    const auto dist_to_stop_pose = autoware::motion_utils::calcSignedArcLength(
      planner_data.value().traj_points, planner_data.value().ego_pose.position,
      stop_pose.value().position);

    Metric dist_to_stop_pose_metric;
    dist_to_stop_pose_metric.name = module_name + "/distance_to_stop_pose";
    dist_to_stop_pose_metric.unit = "double";
    dist_to_stop_pose_metric.value = std::to_string(dist_to_stop_pose);
    metrics.push_back(dist_to_stop_pose_metric);
  }

  if (stop_obstacle.has_value()) {
    // Obstacle info
    Metric collision_point_metric;
    const auto & p = stop_obstacle.value().collision_point;
    collision_point_metric.name = module_name + "/collision_point";
    collision_point_metric.unit = "string";
    collision_point_metric.value =
      "{" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + "}";
    metrics.push_back(collision_point_metric);
  }
  return metrics;
}

void PlannerInterface::publishMetrics(const rclcpp::Time & current_time)
{
  // create array
  MetricArray metrics_msg;
  metrics_msg.stamp = current_time;

  auto addMetrics = [&metrics_msg](std::optional<std::vector<Metric>> & opt_metrics) {
    if (opt_metrics) {
      metrics_msg.metric_array.insert(
        metrics_msg.metric_array.end(), opt_metrics->begin(), opt_metrics->end());
    }
  };
  addMetrics(debug_data_ptr_->stop_metrics);
  addMetrics(debug_data_ptr_->slow_down_metrics);
  addMetrics(debug_data_ptr_->cruise_metrics);
  metrics_pub_->publish(metrics_msg);
  clearMetrics();
}

void PlannerInterface::clearMetrics()
{
  debug_data_ptr_->stop_metrics = std::nullopt;
  debug_data_ptr_->slow_down_metrics = std::nullopt;
  debug_data_ptr_->cruise_metrics = std::nullopt;
}
