/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// #ifndef UTIL_H
// #define UTIL_H
#ifndef OBSTACLE_AVOIDANCE_PLANNER_UTIL_H
#define OBSTACLE_AVOIDANCE_PLANNER_UTIL_H

#include <Eigen/Core>

#include <boost/optional/optional_fwd.hpp>

namespace autoware_planning_msgs
{
ROS_DECLARE_MESSAGE(PathPoint);
ROS_DECLARE_MESSAGE(TrajectoryPoint);
}  // namespace autoware_planning_msgs

struct VehicleParam;
struct ReferencePoint;

namespace util
{
template <typename T>
geometry_msgs::Point transformToRelativeCoordinate2D(
  const T & point, const geometry_msgs::Pose & origin);

geometry_msgs::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin);

double calculate2DDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b);

double calculateSquaredDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b);

double getYawFromPoints(const geometry_msgs::Point & a, const geometry_msgs::Point & a_root);

double normalizeRadian(const double angle);

geometry_msgs::Quaternion getQuaternionFromPoints(
  const geometry_msgs::Point & a, const geometry_msgs::Point & a_root);

template <typename T>
geometry_msgs::Point transformMapToImage(
  const T & map_point, const nav_msgs::MapMetaData & occupancy_grid_info);

boost::optional<geometry_msgs::Point> transformMapToOptionalImage(
  const geometry_msgs::Point & map_point, const nav_msgs::MapMetaData & occupancy_grid_info);

bool transformMapToImage(
  const geometry_msgs::Point & map_point, const nav_msgs::MapMetaData & occupancy_grid_info,
  geometry_msgs::Point & image_point);

bool interpolate2DPoints(
  const std::vector<double> & x, const std::vector<double> & y, const double resolution,
  std::vector<geometry_msgs::Point> & interpolated_points);

std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::Pose> & first_points,
  const std::vector<geometry_msgs::Pose> & second_points, const double delta_arc_length);

std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::Pose> & points, const double delta_arc_length);

std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const std::vector<ReferencePoint> & points, const double delta_arc_length);

template <typename T>
std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const T & points, const double delta_arc_length);

template <typename T>
int getNearestIdx(
  const T & points, const geometry_msgs::Pose & pose, const int default_idx,
  const double delta_yaw_threshold);

int getNearestIdx(
  const std::vector<geometry_msgs::Point> & points, const geometry_msgs::Pose & pose,
  const int default_idx, const double delta_yaw_threshold);

int getNearestIdxOverPoint(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & pose, const int default_idx, const double delta_yaw_threshold);

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::Point & point, const int default_idx);

int getNearestIdx(
  const std::vector<geometry_msgs::Point> & points, const geometry_msgs::Point & point);

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::Point & point);

int getNearestIdx(
  const std::vector<ReferencePoint> & points, const double target_s, const int begin_idx);

template <typename T>
int getNearestPointIdx(const T & points, const geometry_msgs::Point & point);

std::vector<autoware_planning_msgs::TrajectoryPoint> convertPathToTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points);

std::vector<autoware_planning_msgs::TrajectoryPoint> convertPointsToTrajectoryPoinsWithYaw(
  const std::vector<geometry_msgs::Point> & points);

std::vector<autoware_planning_msgs::TrajectoryPoint> fillTrajectoryWithVelocity(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points, const double velocity);

template <typename T>
std::vector<autoware_planning_msgs::TrajectoryPoint> alignVelocityWithPoints(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & traj_points, const T & points,
  const int zero_velocity_traj_idx, const int max_skip_comparison_idx);

struct Rectangle
{
  int min_x_idx = 0;
  int min_y_idx = 0;
  int max_x_idx = 0;
  int max_y_idx = 0;
  int area = 0;
};

std::vector<std::vector<int>> getHistogramTable(const std::vector<std::vector<int>> & input);

Rectangle getLargestRectancleInRow(
  const std::vector<int> & histo, const int curret_row, const int row_size);

Rectangle getLargestRectangle(const std::vector<std::vector<int>> & input);

boost::optional<geometry_msgs::Point> getLastExtendedPoint(
  const autoware_planning_msgs::PathPoint & path_point, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold, const double delta_dist_threshold);

boost::optional<autoware_planning_msgs::TrajectoryPoint> getLastExtendedTrajPoint(
  const autoware_planning_msgs::PathPoint & path_point, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold, const double delta_dist_threshold);

struct Footprint
{
  geometry_msgs::Point p;
  geometry_msgs::Point top_left;
  geometry_msgs::Point top_right;
  geometry_msgs::Point bottom_left;
  geometry_msgs::Point bottom_right;
};

std::vector<Footprint> getVehicleFootprints(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimzied_points,
  const VehicleParam & vehicle_param);

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y);

bool hasValidNearestPointFromEgo(
  const geometry_msgs::Pose & ego_pose, const Trajectories & trajs,
  const TrajectoryParam & traj_param);

std::vector<autoware_planning_msgs::TrajectoryPoint> concatTraj(const Trajectories & trajs);

const int getZeroVelocityIdx(
  const bool is_showing_debug_info, const std::vector<geometry_msgs::Point> & fine_points,
  const std::vector<autoware_planning_msgs::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & opt_trajs, const TrajectoryParam & traj_param);

template <typename T>
int getZeroVelocityIdxFromPoints(
  const T & points, const std::vector<geometry_msgs::Point> & fine_points, const int default_idx,
  const TrajectoryParam & traj_param);

template <typename T>
double getArcLength(const T & points, const int initial_idx = 0);

double getArcLength(const std::vector<geometry_msgs::Pose> & points, const int initial_idx = 0);

void logOSQPSolutionStatus(const int solution_status);

}  // namespace util

#endif
