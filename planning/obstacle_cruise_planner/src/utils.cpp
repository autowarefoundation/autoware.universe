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

#include "obstacle_cruise_planner/utils.hpp"

#include "perception_utils/predicted_path_utils.hpp"

namespace obstacle_cruise_utils
{
visualization_msgs::msg::Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obstacle_pose, size_t idx, const std::string & ns,
  const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();

  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, ns, idx, visualization_msgs::msg::Marker::SPHERE,
    tier4_autoware_utils::createMarkerScale(2.0, 2.0, 2.0),
    tier4_autoware_utils::createMarkerColor(r, g, b, 0.8));

  marker.pose = obstacle_pose;

  return marker;
}

std::optional<geometry_msgs::msg::Pose> calcForwardPose(
  const std::vector<TrajectoryPoint> & traj_points, const size_t start_idx,
  const double target_length)
{
  if (traj_points.empty()) {
    return {};
  }

  size_t search_idx = start_idx;
  double length_to_search_idx = 0.0;
  for (; search_idx < traj_points.size(); ++search_idx) {
    length_to_search_idx = motion_utils::calcSignedArcLength(traj_points, start_idx, search_idx);
    if (length_to_search_idx > target_length) {
      break;
    } else if (search_idx == traj_points.size() - 1) {
      return {};
    }
  }

  if (search_idx == 0 && !traj_points.empty()) {
    return traj_points.at(0).pose;
  }

  const auto & pre_pose = traj_points.at(search_idx - 1).pose;
  const auto & next_pose = traj_points.at(search_idx).pose;

  geometry_msgs::msg::Pose target_pose;

  // lerp position
  const double seg_length =
    tier4_autoware_utils::calcDistance2d(pre_pose.position, next_pose.position);
  const double lerp_ratio = (length_to_search_idx - target_length) / seg_length;

  return tier4_autoware_utils::calcInterpolatedPose(pre_pose, next_pose, lerp_ratio);
}

std::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
  const PredictedPath & predicted_path, const rclcpp::Time & obstacle_base_time,
  const rclcpp::Time & current_time)
{
  const double rel_time = (current_time - obstacle_base_time).seconds();
  if (rel_time < 0.0) {
    return std::nullopt;
  }

  const auto pose = perception_utils::calcInterpolatedPose(predicted_path, rel_time);
  if (!pose) {
    return std::nullopt;
  }
  return pose.get();
}

std::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPaths(
  const std::vector<PredictedPath> & predicted_paths, const rclcpp::Time & obstacle_base_time,
  const rclcpp::Time & current_time)
{
  if (predicted_paths.empty()) {
    return std::nullopt;
  }
  // Get the most reliable path
  const auto predicted_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  return getCurrentObjectPoseFromPredictedPath(*predicted_path, obstacle_base_time, current_time);
}

PoseWithStamp getCurrentObjectPose(
  const PredictedObject & predicted_object, const rclcpp::Time & stamp,
  const rclcpp::Time & current_time, const bool use_prediction)
{
  const auto & pose = predicted_object.kinematics.initial_pose_with_covariance.pose;

  if (!use_prediction) {
    return PoseWithStamp{stamp, pose};
  }

  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  const auto interpolated_pose =
    getCurrentObjectPoseFromPredictedPaths(predicted_paths, stamp, current_time);

  if (!interpolated_pose) {
    RCLCPP_WARN(
      rclcpp::get_logger("ObstacleCruisePlanner"), "Failed to find the interpolated obstacle pose");
    return PoseWithStamp{stamp, pose};
  }

  return PoseWithStamp{stamp, *interpolated_pose};
}

std::optional<StopObstacle> getClosestStopObstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<StopObstacle> & stop_obstacles)
{
  if (stop_obstacles.empty()) {
    return std::nullopt;
  }

  std::optional<StopObstacle> closest_stop_obstacle = std::nullopt;
  double dist_to_closest_stop_obstacle = std::numeric_limits<double>::max();
  for (const auto & stop_obstacle : stop_obstacles) {
    const double dist_to_stop_obstacle =
      motion_utils::calcSignedArcLength(traj_points, 0, stop_obstacle.collision_point);
    if (dist_to_stop_obstacle < dist_to_closest_stop_obstacle) {
      dist_to_closest_stop_obstacle = dist_to_stop_obstacle;
      closest_stop_obstacle = stop_obstacle;
    }
  }
  return closest_stop_obstacle;
}

TrajectoryPoint calcInterpolatedPoint(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose,
  const EgoNearestParam & ego_nearest_param)
{
  return motion_utils::calcInterpolatedPoint(
    motion_utils::convertToTrajectory(traj_points), pose, ego_nearest_param.dist_threshold,
    ego_nearest_param.yaw_threshold);
}
}  // namespace obstacle_cruise_utils
