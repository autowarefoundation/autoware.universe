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

#include "obstacle_cruise_planner/pid_based_planner/pid_based_planner.hpp"

#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "tier4_planning_msgs/msg/velocity_limit.hpp"

namespace
{
VelocityLimit createVelocityLimitMsg(
  const rclcpp::Time & current_time, const double vel, const double acc, const double max_jerk,
  const double min_jerk)
{
  VelocityLimit msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner";
  msg.use_constraints = true;

  msg.max_velocity = vel;
  if (acc < 0) {
    msg.constraints.min_acceleration = acc;
  }
  msg.constraints.max_jerk = max_jerk;
  msg.constraints.min_jerk = min_jerk;

  return msg;
}

Float32MultiArrayStamped convertDebugValuesToMsg(
  const rclcpp::Time & current_time, const DebugValues & debug_values)
{
  Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = current_time;
  for (const auto & v : debug_values.getValues()) {
    debug_msg.data.push_back(v);
  }
  return debug_msg;
}

template <class T>
size_t getIndexWithLongitudinalOffset(
  const T & points, const double longitudinal_offset, boost::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= start_idx.get()) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = start_idx.get(); i < points.size() - 1; ++i) {
      const double segment_length =
        tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double front_length = segment_length;
        const double back_length = sum_length - longitudinal_offset;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = start_idx.get(); i > 0; --i) {
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double front_length = segment_length;
      const double back_length = sum_length + longitudinal_offset;
      if (front_length < back_length) {
        return i;
      } else {
        return i + 1;
      }
    }
  }
  return 0;
}
}  // namespace

PIDBasedPlanner::PIDBasedPlanner(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PlannerInterface(node, longitudinal_info, vehicle_info)
{
  min_accel_during_cruise_ =
    node.declare_parameter<double>("pid_based_planner.min_accel_during_cruise");

  // pid controller
  const double kp = node.declare_parameter<double>("pid_based_planner.kp");
  const double ki = node.declare_parameter<double>("pid_based_planner.ki");
  const double kd = node.declare_parameter<double>("pid_based_planner.kd");
  pid_controller_ = std::make_unique<PIDController>(kp, ki, kd);
  output_ratio_during_accel_ =
    node.declare_parameter<double>("pid_based_planner.output_ratio_during_accel");

  // some parameters
  // use_predicted_obstacle_pose_ =
  //   node.declare_parameter<bool>("pid_based_planner.use_predicted_obstacle_pose");

  vel_to_acc_weight_ = node.declare_parameter<double>("pid_based_planner.vel_to_acc_weight");

  min_cruise_target_vel_ =
    node.declare_parameter<double>("pid_based_planner.min_cruise_target_vel");

  // publisher
  stop_reasons_pub_ =
    node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
  stop_speed_exceeded_pub_ =
    node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
  debug_values_pub_ = node.create_publisher<Float32MultiArrayStamped>("~/debug/values", 1);
}

Trajectory PIDBasedPlanner::generateTrajectory(
  const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);
  debug_values_.resetValues();

  // calc obstacles to cruise and stop
  boost::optional<StopObstacleInfo> stop_obstacle_info;
  boost::optional<CruiseObstacleInfo> cruise_obstacle_info;
  calcObstaclesToCruiseAndStop(planner_data, stop_obstacle_info, cruise_obstacle_info);

  // plan cruise
  planCruise(planner_data, vel_limit, cruise_obstacle_info, debug_data);

  // publish debug values
  publishDebugValues(planner_data);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return planner_data.traj;
}

void PIDBasedPlanner::calcObstaclesToCruiseAndStop(
  const ObstacleCruisePlannerData & planner_data,
  boost::optional<StopObstacleInfo> & stop_obstacle_info,
  boost::optional<CruiseObstacleInfo> & cruise_obstacle_info)
{
  debug_values_.setValues(DebugValues::TYPE::CURRENT_VELOCITY, planner_data.current_vel);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_ACCELERATION, planner_data.current_acc);

  // search highest probability obstacle for stop and cruise
  for (const auto & obstacle : planner_data.target_obstacles) {
    // NOTE: from ego's front to obstacle's back
    const double dist_to_obstacle = calcDistanceToObstacle(planner_data, obstacle);

    const bool is_stop_required = isStopRequired(obstacle);
    if (is_stop_required) {  // stop
      // calculate error distance (= distance to stop)
      const double error_dist = dist_to_obstacle - longitudinal_info_.safe_distance_margin;
      if (stop_obstacle_info) {
        if (error_dist > stop_obstacle_info->dist_to_stop) {
          return;
        }
      }
      stop_obstacle_info = StopObstacleInfo(obstacle, error_dist);

      // update debug values
      debug_values_.setValues(DebugValues::TYPE::STOP_CURRENT_OBJECT_DISTANCE, dist_to_obstacle);
      debug_values_.setValues(DebugValues::TYPE::STOP_CURRENT_OBJECT_VELOCITY, obstacle.velocity);
      debug_values_.setValues(
        DebugValues::TYPE::STOP_TARGET_OBJECT_DISTANCE, longitudinal_info_.safe_distance_margin);
      debug_values_.setValues(
        DebugValues::TYPE::STOP_TARGET_ACCELERATION, longitudinal_info_.limit_min_accel);
      debug_values_.setValues(DebugValues::TYPE::STOP_ERROR_OBJECT_DISTANCE, error_dist);
    } else {  // cruise
      // calculate distance between ego and obstacle based on RSS
      const double rss_dist = calcRSSDistance(
        planner_data.current_vel, obstacle.velocity, longitudinal_info_.safe_distance_margin);

      // calculate error distance and normalized one
      const double error_dist = dist_to_obstacle - rss_dist;
      if (cruise_obstacle_info) {
        if (error_dist > cruise_obstacle_info->dist_to_cruise) {
          return;
        }
      }
      const double normalized_dist_to_cruise = error_dist / dist_to_obstacle;
      cruise_obstacle_info = CruiseObstacleInfo(obstacle, error_dist, normalized_dist_to_cruise);

      // update debug values
      debug_values_.setValues(DebugValues::TYPE::CRUISE_CURRENT_OBJECT_VELOCITY, obstacle.velocity);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_CURRENT_OBJECT_DISTANCE, dist_to_obstacle);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_TARGET_OBJECT_DISTANCE, rss_dist);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_ERROR_OBJECT_DISTANCE, error_dist);
    }
  }
}

double PIDBasedPlanner::calcDistanceToObstacle(
  const ObstacleCruisePlannerData & planner_data, const TargetObstacle & obstacle)
{
  // TODO(murooka) enable this option considering collision_point (precise obstacle point to measure
  // distance) if (use_predicted_obstacle_pose_) {
  //   // interpolate current obstacle pose from predicted path
  //   const auto current_interpolated_obstacle_pose =
  //     obstacle_cruise_utils::getCurrentObjectPoseFromPredictedPath(
  //       obstacle.predicted_paths.at(0), obstacle.time_stamp, planner_data.current_time);
  //   if (current_interpolated_obstacle_pose) {
  //     return tier4_autoware_utils::calcSignedArcLength(
  //       planner_data.traj.points, planner_data.current_pose.position,
  //       current_interpolated_obstacle_pose->position) - offset;
  //   }
  //
  //   RCLCPP_INFO_EXPRESSION(
  //     rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), true,
  //     "Failed to interpolated obstacle pose from predicted path. Use non-interpolated obstacle
  //     pose.");
  // }

  const size_t ego_segment_idx =
    findExtendedNearestSegmentIndex(planner_data.traj, planner_data.current_pose);
  const double segment_offset = std::max(
    0.0, tier4_autoware_utils::calcLongitudinalOffsetToSegment(
           planner_data.traj.points, ego_segment_idx, planner_data.current_pose.position));
  const double offset = vehicle_info_.max_longitudinal_offset_m + segment_offset;

  return tier4_autoware_utils::calcSignedArcLength(
           planner_data.traj.points, ego_segment_idx, obstacle.collision_point) -
         offset;
}

void PIDBasedPlanner::planCruise(
  const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
  const boost::optional<CruiseObstacleInfo> & cruise_obstacle_info, DebugData & debug_data)
{
  // do cruise
  if (cruise_obstacle_info) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
      "cruise planning");

    vel_limit = doCruise(
      planner_data, cruise_obstacle_info.get(), debug_data.obstacles_to_cruise,
      debug_data.cruise_wall_marker);

    // update debug values
    debug_values_.setValues(DebugValues::TYPE::CRUISE_TARGET_VELOCITY, vel_limit->max_velocity);
    debug_values_.setValues(
      DebugValues::TYPE::CRUISE_TARGET_ACCELERATION, vel_limit->constraints.min_acceleration);
  } else {
    // reset previous target velocity if adaptive cruise is not enabled
    prev_target_vel_ = {};
  }
}

VelocityLimit PIDBasedPlanner::doCruise(
  const ObstacleCruisePlannerData & planner_data, const CruiseObstacleInfo & cruise_obstacle_info,
  std::vector<TargetObstacle> & debug_obstacles_to_cruise,
  visualization_msgs::msg::MarkerArray & debug_wall_marker)
{
  const double dist_to_cruise = cruise_obstacle_info.dist_to_cruise;
  const double normalized_dist_to_cruise = cruise_obstacle_info.normalized_dist_to_cruise;

  const size_t ego_idx = findExtendedNearestIndex(planner_data.traj, planner_data.current_pose);

  // calculate target velocity with acceleration limit by PID controller
  const double pid_output_vel = pid_controller_->calc(normalized_dist_to_cruise);
  [[maybe_unused]] const double prev_vel =
    prev_target_vel_ ? prev_target_vel_.get() : planner_data.current_vel;

  const double additional_vel = [&]() {
    if (normalized_dist_to_cruise > 0) {
      return pid_output_vel * output_ratio_during_accel_;
    }
    return pid_output_vel;
  }();

  const double positive_target_vel =
    std::max(min_cruise_target_vel_, planner_data.current_vel + additional_vel);

  // calculate target acceleration
  const double target_acc = vel_to_acc_weight_ * additional_vel;
  const double target_acc_with_acc_limit =
    std::clamp(target_acc, min_accel_during_cruise_, longitudinal_info_.max_accel);

  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
    "target_velocity %f", positive_target_vel);

  prev_target_vel_ = positive_target_vel;

  // set target longitudinal motion
  const auto vel_limit = createVelocityLimitMsg(
    planner_data.current_time, positive_target_vel, target_acc_with_acc_limit,
    longitudinal_info_.max_jerk, longitudinal_info_.min_jerk);

  // virtual wall marker for cruise
  const double dist_to_rss_wall = dist_to_cruise + vehicle_info_.max_longitudinal_offset_m;
  const size_t wall_idx =
    getIndexWithLongitudinalOffset(planner_data.traj.points, dist_to_rss_wall, ego_idx);

  const auto markers = tier4_autoware_utils::createSlowDownVirtualWallMarker(
    planner_data.traj.points.at(wall_idx).pose, "obstacle cruise", planner_data.current_time, 0);
  tier4_autoware_utils::appendMarkerArray(markers, &debug_wall_marker);

  debug_obstacles_to_cruise.push_back(cruise_obstacle_info.obstacle);

  return vel_limit;
}

void PIDBasedPlanner::publishDebugValues(const ObstacleCruisePlannerData & planner_data) const
{
  const auto debug_values_msg = convertDebugValuesToMsg(planner_data.current_time, debug_values_);
  debug_values_pub_->publish(debug_values_msg);
}

void PIDBasedPlanner::updateParam(const std::vector<rclcpp::Parameter> & parameters)
{
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.min_accel_during_cruise", min_accel_during_cruise_);

  // pid controller
  double kp = pid_controller_->getKp();
  double ki = pid_controller_->getKi();
  double kd = pid_controller_->getKd();

  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.kp", kp);
  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.ki", ki);
  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.kd", kd);
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.output_ratio_during_accel", output_ratio_during_accel_);

  // vel_to_acc_weight
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.vel_to_acc_weight", vel_to_acc_weight_);

  // min_cruise_target_vel
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.min_cruise_target_vel", min_cruise_target_vel_);

  pid_controller_->updateParam(kp, ki, kd);
}
