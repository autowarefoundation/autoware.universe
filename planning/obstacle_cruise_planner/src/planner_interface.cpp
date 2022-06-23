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

#include "obstacle_cruise_planner/planner_interface.hpp"

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

tier4_planning_msgs::msg::StopReasonArray makeStopReasonArray(
  const rclcpp::Time & current_time, const geometry_msgs::msg::Pose & stop_pose,
  const TargetObstacle & stop_obstacle)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop factor
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose;
  stop_factor.stop_factor_points.emplace_back(stop_obstacle.collision_point);

  // create stop reason stamped
  tier4_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP;
  stop_reason_msg.stop_factors.emplace_back(stop_factor);

  // create stop reason array
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

double calcMinimumDistanceToStop(const double initial_vel, const double min_acc)
{
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}
}  // namespace

boost::optional<TargetObstacle> PlannerInterface::getClosestStopObstacle(
  const Trajectory & traj, const std::vector<TargetObstacle> & target_obstacles)
{
  if (target_obstacles.empty()) {
    return boost::none;
  }

  TargetObstacle closest_stop_obstacle = target_obstacles.front();
  double dist_to_closest_stop_obstacle = std::numeric_limits<double>::max();
  for (const auto & obstacle : target_obstacles) {
    // Ignore obstacle that is not required to stop
    if (!isStopRequired(obstacle)) {
      continue;
    }

    const double dist_to_stop_obstacle =
      tier4_autoware_utils::calcSignedArcLength(traj.points, 0, obstacle.collision_point);
    if (dist_to_stop_obstacle < dist_to_closest_stop_obstacle) {
      dist_to_closest_stop_obstacle = dist_to_stop_obstacle;
      closest_stop_obstacle = obstacle;
    }
  }

  return closest_stop_obstacle;
}

Trajectory PlannerInterface::generateStopTrajectory(
  const Trajectory & traj, const geometry_msgs::msg::Pose & current_pose, const double current_vel,
  const std::vector<TargetObstacle> & target_obstacles, const rclcpp::Time & current_time,
  DebugData & debug_data)
{
  if (target_obstacles.empty()) {
    return traj;
  }

  const auto traj_to_ego = tier4_autoware_utils::calcSignedArcLength(
    traj.points, current_pose, 0, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!traj_to_ego) {
    return traj;
  }

  // Get Closest Behavior Stop Distance
  const double traj_length = tier4_autoware_utils::calcArcLength(traj.points);
  const auto closest_forward_stop_dist = tier4_autoware_utils::calcDistanceToForwardStopPoint(
    traj.points, current_pose, nearest_dist_deviation_threshold_, nearest_yaw_deviation_threshold_);
  const double closest_behavior_stop_dist =
    closest_forward_stop_dist ? std::min(*traj_to_ego + *closest_forward_stop_dist, traj_length)
                              : traj_length;

  // Get Closest Stop Obstacle
  const auto closest_stop_obstacle = getClosestStopObstacle(traj, target_obstacles);
  if (!closest_stop_obstacle) {
    // No stop obstacle
    return traj;
  }

  // Get Closest Obstacle Stop Distance
  const double offset = vehicle_info_.max_longitudinal_offset_m + min_behavior_stop_margin_;
  const double closest_obstacle_stop_dist =
    tier4_autoware_utils::calcSignedArcLength(
      traj.points, 0, closest_stop_obstacle->collision_point) -
    offset;

  if (closest_behavior_stop_dist < closest_obstacle_stop_dist) {
    // Original Stop Point on the trajectory is closer to the stop obstacle
    return traj;
  }

  // If behavior stop point is ahead of the closest_obstacle_stop point within a certain margin
  // we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const double stop_dist_diff = closest_behavior_stop_dist - closest_obstacle_stop_dist;
  if (0 < stop_dist_diff && stop_dist_diff < longitudinal_info_.safe_distance_margin) {
    // Use original stop point on the trajectory
    return traj;
  }

  // Calculate closest stop distance (Check the feasibility)
  const double feasible_dist_to_stop_from_ego =
    calcMinimumDistanceToStop(current_vel, longitudinal_info_.limit_min_accel);
  const double closest_obstacle_stop_dist_from_ego = closest_obstacle_stop_dist - *traj_to_ego;

  bool will_collide_with_obstacle = false;
  double closest_stop_dist = closest_obstacle_stop_dist;
  if (closest_obstacle_stop_dist_from_ego < feasible_dist_to_stop_from_ego) {
    closest_stop_dist = *traj_to_ego + feasible_dist_to_stop_from_ego;
    will_collide_with_obstacle = true;
  }

  // Generate Output Trajectory
  const auto output_traj = obstacle_cruise_utils::insertStopPoint(traj, closest_stop_dist);

  /***********
   ** Debug **
   ***********/
  // virtual wall marker for stop obstacle
  const auto marker_pose = obstacle_cruise_utils::calcForwardPose(
    traj, 0, closest_stop_dist + vehicle_info_.max_longitudinal_offset_m);
  if (marker_pose) {
    const auto markers = tier4_autoware_utils::createStopVirtualWallMarker(
      marker_pose.get(), "obstacle stop", current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);
  }
  debug_data.obstacles_to_stop.push_back(*closest_stop_obstacle);

  // Publish Stop Reason
  const auto stop_idx = tier4_autoware_utils::searchZeroVelocityIndex(traj.points);
  if (stop_idx) {
    const auto stop_pose = traj.points.at(*stop_idx).pose;
    const auto stop_reasons_msg =
      makeStopReasonArray(current_time, stop_pose, *closest_stop_obstacle);
    stop_reasons_pub_->publish(stop_reasons_msg);
  }

  // Publish if ego vehicle collides with the obstacle with a limit acceleration
  const auto stop_speed_exceeded_msg =
    createStopSpeedExceededMsg(current_time, will_collide_with_obstacle);
  stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);

  return output_traj;
}
