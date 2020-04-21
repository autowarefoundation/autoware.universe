/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 */

#include <stack>

#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>

#include <tf2/utils.h>

#include <Eigen/Core>

#include "obstacle_avoidance_planner/spline_interpolate.h"
#include "obstacle_avoidance_planner/util.h"

namespace util
{
template <typename T>
geometry_msgs::Point transformToRelativeCoordinate2D(
  const T & point, const geometry_msgs::Pose & origin)
{
  geometry_msgs::Transform origin_coord2point;
  origin_coord2point.translation.x = point.x;
  origin_coord2point.translation.y = point.y;
  tf2::Transform tf_origin_coord2point;
  tf2::fromMsg(origin_coord2point, tf_origin_coord2point);

  geometry_msgs::Transform origin_coord2origin;
  origin_coord2origin.translation.x = origin.position.x;
  origin_coord2origin.translation.y = origin.position.y;
  origin_coord2origin.rotation = origin.orientation;
  tf2::Transform tf_origin_coord2origin;
  tf2::fromMsg(origin_coord2origin, tf_origin_coord2origin);

  tf2::Transform tf_origin2origin_coord = tf_origin_coord2origin.inverse() * tf_origin_coord2point;
  geometry_msgs::Pose rel_pose;
  tf2::toMsg(tf_origin2origin_coord, rel_pose);
  geometry_msgs::Point relative_p = rel_pose.position;
  return relative_p;
}
template geometry_msgs::Point transformToRelativeCoordinate2D<geometry_msgs::Point>(
  const geometry_msgs::Point &, const geometry_msgs::Pose & origin);
template geometry_msgs::Point transformToRelativeCoordinate2D<geometry_msgs::Point32>(
  const geometry_msgs::Point32 &, const geometry_msgs::Pose & origin);

geometry_msgs::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin)
{
  geometry_msgs::Transform origin2point;
  origin2point.translation.x = point.x;
  origin2point.translation.y = point.y;
  tf2::Transform tf_origin2point;
  tf2::fromMsg(origin2point, tf_origin2point);

  geometry_msgs::Transform origin_coord2origin;
  origin_coord2origin.translation.x = origin.position.x;
  origin_coord2origin.translation.y = origin.position.y;
  origin_coord2origin.rotation = origin.orientation;
  tf2::Transform tf_origin_coord2origin;
  tf2::fromMsg(origin_coord2origin, tf_origin_coord2origin);
  tf2::Transform tf_origin_coord2point = tf_origin_coord2origin * tf_origin2point;

  geometry_msgs::Pose abs_pose;
  tf2::toMsg(tf_origin_coord2point, abs_pose);
  geometry_msgs::Point abs_p = abs_pose.position;
  return abs_p;
}

double calculate2DDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calculateSquaredDistance(const geometry_msgs::Point & a, const geometry_msgs::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double getYawFromPoints(const geometry_msgs::Point & a, const geometry_msgs::Point & a_root)
{
  const double dx = a.x - a_root.x;
  const double dy = a.y - a_root.y;
  return std::atan2(dy, dx);
}

double normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

geometry_msgs::Quaternion getQuaternionFromPoints(
  const geometry_msgs::Point & a, const geometry_msgs::Point & a_root)
{
  const double roll = 0;
  const double pitch = 0;
  const double yaw = util::getYawFromPoints(a, a_root);
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quaternion);
}

template <typename T>
geometry_msgs::Point transformMapToImage(
  const T & map_point, const nav_msgs::MapMetaData & occupancy_grid_info)
{
  geometry_msgs::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x / resolution;
  double map_y_in_image_resolution = relative_p.y / resolution;
  geometry_msgs::Point image_point;
  image_point.x = map_y_height - map_y_in_image_resolution;
  image_point.y = map_x_width - map_x_in_image_resolution;
  return image_point;
}
template geometry_msgs::Point transformMapToImage<geometry_msgs::Point>(
  const geometry_msgs::Point &, const nav_msgs::MapMetaData & map_info);
template geometry_msgs::Point transformMapToImage<geometry_msgs::Point32>(
  const geometry_msgs::Point32 &, const nav_msgs::MapMetaData & map_info);

std::shared_ptr<geometry_msgs::Point> transformMapToImagePtr(
  const geometry_msgs::Point & map_point, const nav_msgs::MapMetaData & occupancy_grid_info)
{
  geometry_msgs::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x / resolution;
  double map_y_in_image_resolution = relative_p.y / resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
  if (image_x >= 0 && image_x < (int)map_y_height && image_y >= 0 && image_y < (int)map_x_width) {
    geometry_msgs::Point image_point;
    image_point.x = image_x;
    image_point.y = image_y;
    std::shared_ptr<geometry_msgs::Point> image_point_ptr;
    image_point_ptr = std::make_shared<geometry_msgs::Point>(image_point);
    return image_point_ptr;
  } else {
    return nullptr;
  }
}

bool transformMapToImage(
  const geometry_msgs::Point & map_point, const nav_msgs::MapMetaData & occupancy_grid_info,
  geometry_msgs::Point & image_point)
{
  geometry_msgs::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  const double map_y_height = occupancy_grid_info.height;
  const double map_x_width = occupancy_grid_info.width;
  const double scale = 1 / occupancy_grid_info.resolution;
  const double map_x_in_image_resolution = relative_p.x * scale;
  const double map_y_in_image_resolution = relative_p.y * scale;
  const double image_x = map_y_height - map_y_in_image_resolution;
  const double image_y = map_x_width - map_x_in_image_resolution;
  if (image_x >= 0 && image_x < (int)map_y_height && image_y >= 0 && image_y < (int)map_x_width) {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  } else {
    return false;
  }
}

bool interpolate2DPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution,
  std::vector<geometry_msgs::Point> & interpolated_points)
{
  if (base_x.empty() || base_y.empty()) {
    return false;
  }
  std::vector<double> base_s = spline::calcEuclidDist(base_x, base_y);
  if (base_s.empty()) {
    return false;
  }
  std::vector<double> new_s;
  for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }
  // new_s.push_back(base_s.back());
  spline::SplineInterpolate spline;
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  spline.interpolate(base_s, base_x, new_s, interpolated_x);
  spline.interpolate(base_s, base_y, new_s, interpolated_y);
  for (size_t i = 0; i < interpolated_x.size(); i++) {
    geometry_msgs::Point point;
    point.x = interpolated_x[i];
    point.y = interpolated_y[i];
    interpolated_points.push_back(point);
  }
}

std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::Pose> & first_points,
  const std::vector<geometry_msgs::Pose> & second_points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (const auto point : first_points) {
    tmp_x.push_back(point.position.x);
    tmp_y.push_back(point.position.y);
  }
  for (const auto & point : second_points) {
    tmp_x.push_back(point.position.x);
    tmp_y.push_back(point.position.y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::Pose> & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (const auto & point : points) {
    tmp_x.push_back(point.position.x);
    tmp_y.push_back(point.position.y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

template <typename T>
std::vector<geometry_msgs::Point> getInterpolatedPoints(
  const T & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (const auto & point : points) {
    tmp_x.push_back(point.pose.position.x);
    tmp_y.push_back(point.pose.position.y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}
template std::vector<geometry_msgs::Point>
getInterpolatedPoints<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const double);
template std::vector<geometry_msgs::Point>
getInterpolatedPoints<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const double);

template <typename T>
int getNearestIdx(
  const T & points, const geometry_msgs::Pose & pose, const int default_idx,
  const double delta_yaw_threshold)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  const double point_yaw = tf2::getYaw(pose.orientation);
  for (int i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i].pose.position, pose.position);
    const double points_yaw = tf2::getYaw(points[i].pose.orientation);
    const double diff_yaw = points_yaw - point_yaw;
    const double norm_diff_yaw = normalizeRadian(diff_yaw);
    if (dist < min_dist && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
template int getNearestIdx<std::vector<autoware_planning_msgs::TrajectoryPoint>>(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> &, const geometry_msgs::Pose &,
  const int, const double);
template int getNearestIdx<std::vector<autoware_planning_msgs::PathPoint>>(
  const std::vector<autoware_planning_msgs::PathPoint> &, const geometry_msgs::Pose &, const int,
  const double);

int getNearestIdx(
  const std::vector<geometry_msgs::Point> & points, const geometry_msgs::Pose & pose,
  const int default_idx, const double delta_yaw_threshold)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  const double point_yaw = tf2::getYaw(pose.orientation);
  for (int i = 0; i < points.size(); i++) {
    if (i > 0) {
      const double dist = calculateSquaredDistance(points[i], pose.position);
      const double points_yaw = getYawFromPoints(points[i], points[i - 1]);
      const double diff_yaw = points_yaw - point_yaw;
      const double norm_diff_yaw = normalizeRadian(diff_yaw);
      if (dist < min_dist && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
  }
  return nearest_idx;
}

int getNearestIdx(
  const std::vector<autoware_planning_msgs::PathPoint> & points, const geometry_msgs::Pose & pose,
  const int default_idx)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  for (int i = 0; i < points.size(); i++) {
    if (i > 0) {
      const double dist = calculateSquaredDistance(points[i - 1].pose.position, pose.position);
      const double points_dx = points[i].pose.position.x - points[i - 1].pose.position.x;
      const double points_dy = points[i].pose.position.y - points[i - 1].pose.position.y;
      const double points2pose_dx = pose.position.x - points[i - 1].pose.position.x;
      const double points2pose_dy = pose.position.y - points[i - 1].pose.position.y;
      const double ip = points_dx * points2pose_dx + points_dy * points2pose_dy;
      if (ip < 0) {
        return nearest_idx;
      }
      if (dist < min_dist && ip > 0) {
        min_dist = dist;
        nearest_idx = i - 1;
      }
    }
  }
  return nearest_idx;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> convertPathToTrajectory(
  const std::vector<autoware_planning_msgs::PathPoint> & path_points)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
  for (const auto & point : path_points) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose = point.pose;
    tmp_point.twist = point.twist;
    traj_points.push_back(tmp_point);
  }
  return traj_points;
}

std::vector<std::vector<int>> getHistogramTable(const std::vector<std::vector<int>> & input)
{
  std::vector<std::vector<int>> histogram_table = input;
  for (int i = 0; i < input.size(); i++) {
    for (int j = 0; j < input[i].size(); j++) {
      if (input[i][j]) {
        histogram_table[i][j] = 0;
      } else {
        histogram_table[i][j] = (i > 0) ? histogram_table[i - 1][j] + 1 : 1;
      }
    }
  }
  return histogram_table;
}

struct HistogramBin
{
  int height;
  int variable_pos;
  int original_pos;
};

Rectangle getLargestRectancleInRow(
  const std::vector<int> & histo, const int curret_row, const int row_size)
{
  std::vector<int> search_histo = histo;
  search_histo.push_back(0);
  std::stack<HistogramBin> stack;
  Rectangle largest_rect;
  for (int i = 0; i < search_histo.size(); i++) {
    HistogramBin bin;
    bin.height = search_histo[i];
    bin.variable_pos = i;
    bin.original_pos = i;
    if (stack.empty()) {
      stack.push(bin);
    } else {
      if (stack.top().height < bin.height) {
        stack.push(bin);
      } else if (stack.top().height >= bin.height) {
        int target_i = i;
        while (!stack.empty() && bin.height <= stack.top().height) {
          HistogramBin tmp_bin = stack.top();
          stack.pop();
          int area = (i - tmp_bin.variable_pos) * tmp_bin.height;
          if (area > largest_rect.area) {
            largest_rect.max_y_idx = tmp_bin.variable_pos;
            largest_rect.min_y_idx = i - 1;
            largest_rect.max_x_idx = curret_row - tmp_bin.height + 1;
            largest_rect.min_x_idx = curret_row;
            largest_rect.area = area;
          }

          target_i = tmp_bin.variable_pos;
        }
        bin.variable_pos = target_i;
        stack.push(bin);
      }
    }
  }
  return largest_rect;
}

Rectangle getLargestRectangle(const std::vector<std::vector<int>> & input)
{
  std::vector<std::vector<int>> histogram_table = getHistogramTable(input);
  Rectangle largest_rectangle;
  for (int i = 0; i < histogram_table.size(); i++) {
    Rectangle rect = getLargestRectancleInRow(histogram_table[i], i, input.size());
    if (rect.area > largest_rectangle.area) {
      largest_rectangle = rect;
    }
  }
  return largest_rectangle;
}
}  // namespace util
