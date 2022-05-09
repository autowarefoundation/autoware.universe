// Copyright 2022 Tier IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_PLANNER__SAMPLING_BASED_PLANNER__SAMPLING_BASED_PLANNER_HPP_
#define OBSTACLE_VELOCITY_PLANNER__SAMPLING_BASED_PLANNER__SAMPLING_BASED_PLANNER_HPP_

#include "obstacle_velocity_planner/planner_interface.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <limits>

#include <frenet_planner/polynomials.hpp>


class SamplingBasedPlanner : public PlannerInterface {
private:
  // ROS param
  double vel_to_acc_weight_;
  double min_cruise_target_vel_;
  double additional_safe_distance_margin_;
  double max_vehicle_obj_velocity_to_stop_;
  double max_non_vehicle_obj_velocity_to_stop_;
public:
  Trajectory generateTrajectory(
    const ObstacleVelocityPlannerData & planner_data,
    boost::optional<VelocityLimit> & vel_limit) override
{
  auto closest_dist_to_obstacle = std::numeric_limits<double>::max();
  auto target_final_dist_to_obstacle = 0.0;
  auto obstacle_velocity = 0.0;
  for (const auto & obstacle : planner_data.target_obstacles) {
    const double dist_to_obstacle = tier4_autoware_utils::calcSignedArcLength(
      planner_data.traj.points, planner_data.current_pose.position, obstacle.pose.position);
    if(dist_to_obstacle < closest_dist_to_obstacle) {
      closest_dist_to_obstacle = dist_to_obstacle;
      const auto rss_dist =
        calcRSSDistance(planner_data.current_vel, obstacle.velocity, longitudinal_param_.safe_distance_margin + additional_safe_distance_margin_);
      target_final_dist_to_obstacle = rss_dist + vehicle_info_.max_longitudinal_offset_m + obstacle.shape.dimensions.x / 2.0;
      obstacle_velocity = obstacle.velocity;
    }
  }

  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleVelocityPlanner::SamplingBasedPlanner"), true, "slow down planning");

  // calculate min/max range of durations to catchup to the preceding vehicle.
  auto min_duration = 0.0;
  auto max_duration = 0.0;
  // bangbang: max acceleration + constant vel + slow down to obstacle velocity with max deceleration
  // t = (v_target - v_init) / a
  const auto & cur_vel = vel_limit->max_velocity;
  const auto & max_vel = vel_limit->max_velocity;
  const auto & min_acc = longitudinal_param_.min_accel;
  const auto & max_acc = longitudinal_param_.max_accel;
  auto time_to_max_vel = std::abs(max_vel - cur_vel) / max_acc;
  auto time_max_vel_to_obs_vel = std::abs(obstacle_velocity - max_vel) / -min_acc;
  auto dist_to_max_vel = cur_vel * time_to_max_vel + 0.5 * max_acc * time_to_max_vel * time_to_max_vel;
  auto dist_max_vel_to_obs_vel = max_vel * time_max_vel_to_obs_vel + 0.5 * -min_acc * time_max_vel_to_obs_vel * time_max_vel_to_obs_vel;
  const auto time_bangbang = time_to_max_vel + time_max_vel_to_obs_vel;
  const auto dist_bangbang = dist_to_max_vel + dist_max_vel_to_obs_vel;
  if(dist_bangbang < time_bangbang * obstacle_velocity + closest_dist_to_obstacle) {
    // add time at constant max velocity
    const auto dist_diff = time_bangbang * obstacle_velocity + closest_dist_to_obstacle - dist_bangbang;
    const auto time_const_max_vel = dist_diff / (max_vel - obstacle_velocity);
    min_duration = time_bangbang + time_const_max_vel;
  }
  else {
    // bang bang at a reduced velocity
  }
  if(obstacle_velocity < cur_vel) {
    // max: slow down to obstacle velocity with constant deceleration
    // min: (bangbang) max acceleration + constant vel + slow down to obstacle velocity with max deceleration
  }
  else {
  }
  // sample velocity profiles
  constexpr auto nb_samples = 10;
  constexpr auto target_acc = 0.0;
  const auto target_vel = obstacle_velocity;
  const auto duration_step = (max_duration - min_duration) / (nb_samples - 1);
  for(auto target_duration = min_duration; target_duration <= max_duration; target_duration += duration_step) {
    const auto target_pos = closest_dist_to_obstacle + target_duration * obstacle_velocity - target_final_dist_to_obstacle;
    const auto velocity_polynomial = frenet_planner::Polynomial(
        0.0, cur_vel, planner_data.current_acc, target_pos, target_vel, target_acc, target_duration);
  }
  // select best valid velocity profile

  auto output_traj = planner_data.traj;
  return output_traj;
}

  void updateParam(const std::vector<rclcpp::Parameter> & parameters) override {}
};

#endif // OBSTACLE_VELOCITY_PLANNER__SAMPLING_BASED_PLANNER__SAMPLING_BASED_PLANNER_HPP_