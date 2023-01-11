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

#include "collision_free_path_planner/utils/trajectory_utils.hpp"

#include "collision_free_path_planner/eb_path_optimizer.hpp"
#include "collision_free_path_planner/mpt_optimizer.hpp"
#include "motion_utils/motion_utils.hpp"
#include "tf2/utils.h"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <stack>
#include <vector>

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const collision_free_path_planner::ReferencePoint & p)
{
  return p.p;
}

template <>
geometry_msgs::msg::Pose getPose(const collision_free_path_planner::ReferencePoint & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position = p.p;
  pose.orientation = createQuaternionFromYaw(p.yaw);
  return pose;
}
}  // namespace tier4_autoware_utils

namespace collision_free_path_planner
{
namespace trajectory_utils
{
// functions to convert to another type of points
std::vector<geometry_msgs::msg::Pose> convertToPosesWithYawEstimation(
  const std::vector<geometry_msgs::msg::Point> points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (points.empty()) {
    return poses;
  } else if (points.size() == 1) {
    geometry_msgs::msg::Pose pose;
    pose.position = points.at(0);
    poses.push_back(pose);
    return poses;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position = points.at(i);

    const size_t front_idx = (i == points.size() - 1) ? i - 1 : i;
    const double points_yaw =
      tier4_autoware_utils::calcAzimuthAngle(points.at(front_idx), points.at(front_idx + 1));
    pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(points_yaw);

    poses.push_back(pose);
  }
  return poses;
}

ReferencePoint convertToReferencePoint(const TrajectoryPoint & traj_point)
{
  ReferencePoint ref_point;

  ref_point.p = traj_point.pose.position;
  ref_point.yaw = tf2::getYaw(traj_point.pose.orientation);
  ref_point.v = traj_point.longitudinal_velocity_mps;

  return ref_point;
}

std::vector<ReferencePoint> convertToReferencePoints(
  const std::vector<TrajectoryPoint> & traj_points)
{
  std::vector<ReferencePoint> ref_points;
  for (const auto & traj_point : traj_points) {
    const auto ref_point = convertToReferencePoint(traj_point);
    ref_points.push_back(ref_point);
  }

  return ref_points;
}

void compensateLastPose(
  const PathPoint & last_path_point, std::vector<TrajectoryPoint> & traj_points,
  const double delta_dist_threshold, const double delta_yaw_threshold)
{
  if (traj_points.empty()) {
    traj_points.push_back(convertToTrajectoryPoint(last_path_point));
    return;
  }

  const geometry_msgs::msg::Pose last_traj_pose = traj_points.back().pose;

  const double dist =
    tier4_autoware_utils::calcDistance2d(last_path_point.pose.position, last_traj_pose.position);
  const double norm_diff_yaw = [&]() {
    const double diff_yaw =
      tf2::getYaw(last_path_point.pose.orientation) - tf2::getYaw(last_traj_pose.orientation);
    return tier4_autoware_utils::normalizeRadian(diff_yaw);
  }();
  if (dist > delta_dist_threshold || std::fabs(norm_diff_yaw) > delta_yaw_threshold) {
    traj_points.push_back(convertToTrajectoryPoint(last_path_point));
  }
}

geometry_msgs::msg::Point getNearestPosition(
  const std::vector<ReferencePoint> & points, const int target_idx, const double offset)
{
  double sum_arc_length = 0.0;
  for (size_t i = target_idx; i < points.size(); ++i) {
    sum_arc_length += points.at(target_idx).delta_arc_length;

    if (offset < sum_arc_length) {
      return points.at(target_idx).p;
    }
  }

  return points.back().p;
}

Trajectory createTrajectory(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & traj_points)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> traj_points, const double interval)
{
  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(traj, interval);
  return motion_utils::convertToTrajectoryPointArray(resampled_traj);
}
}  // namespace trajectory_utils
}  // namespace collision_free_path_planner
