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

namespace autoware_planning_msgs
{
ROS_DECLARE_MESSAGE(PathPoint);
ROS_DECLARE_MESSAGE(TrajectoryPoint);
}  // namespace autoware_planning_msgs

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

std::shared_ptr<geometry_msgs::Point> transformMapToImagePtr(
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

int getNearestIdx(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const geometry_msgs::Pose & pose,
  const int default_idx);

std::vector<autoware_planning_msgs::TrajectoryPoint> convertPathToTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points);

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
}  // namespace util

#endif
