// Copyright 2025 TIER IV, Inc.
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

#include "autoware/motion_velocity_planner_common_universe/polygon_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::polygon_utils
{
namespace
{
PointWithStamp calc_nearest_collision_point(
  const size_t first_within_idx, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const bool is_driving_forward)
{
  const size_t prev_idx = first_within_idx == 0 ? first_within_idx : first_within_idx - 1;
  const size_t next_idx = first_within_idx == 0 ? first_within_idx + 1 : first_within_idx;

  std::vector<geometry_msgs::msg::Pose> segment_points{
    decimated_traj_points.at(prev_idx).pose, decimated_traj_points.at(next_idx).pose};
  if (!is_driving_forward) {
    std::reverse(segment_points.begin(), segment_points.end());
  }

  std::vector<double> dist_vec;
  for (const auto & collision_point : collision_points) {
    const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      segment_points, 0, collision_point.point);
    dist_vec.push_back(dist);
  }

  const size_t min_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
  return collision_points.at(min_idx);
}

// NOTE: max_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::optional<std::pair<size_t, std::vector<PointWithStamp>>> get_collision_index(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose & object_pose, const rclcpp::Time & object_time,
  const Shape & object_shape, const double max_dist = std::numeric_limits<double>::max())
{
  const auto obj_polygon = autoware_utils::to_polygon2d(object_pose, object_shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      autoware_utils::calc_distance2d(traj_points.at(i).pose, object_pose);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    std::vector<PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          PointWithStamp collision_geom_point;
          collision_geom_point.stamp = object_time;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_point.point.z = traj_points.at(i).pose.position.z;
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      const auto collision_info =
        std::pair<size_t, std::vector<PointWithStamp>>{i, collision_geom_points};
      return collision_info;
    }
  }

  return std::nullopt;
}
}  // namespace

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose obj_pose, const rclcpp::Time obj_stamp, const Shape & obj_shape,
  const double dist_to_bumper)
{
  const auto collision_info =
    get_collision_index(traj_points, traj_polygons, obj_pose, obj_stamp, obj_shape);
  if (!collision_info) {
    return std::nullopt;
  }

  const auto bumper_pose = autoware_utils::calc_offset_pose(
    traj_points.at(collision_info->first).pose, dist_to_bumper, 0.0, 0.0);

  std::optional<double> max_collision_length = std::nullopt;
  std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
  for (const auto & poly_vertex : collision_info->second) {
    const double dist_from_bumper =
      std::abs(autoware_utils::inverse_transform_point(poly_vertex.point, bumper_pose).x);

    if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  return std::make_pair(
    *max_collision_point,
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, collision_info->first) -
      *max_collision_length);
}

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const size_t collision_idx,
  const std::vector<PointWithStamp> & collision_points, const double dist_to_bumper)
{
  std::pair<size_t, std::vector<PointWithStamp>> collision_info;
  collision_info.first = collision_idx;
  collision_info.second = collision_points;

  const auto bumper_pose = autoware_utils::calc_offset_pose(
    traj_points.at(collision_info.first).pose, dist_to_bumper, 0.0, 0.0);

  std::optional<double> max_collision_length = std::nullopt;
  std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
  for (const auto & poly_vertex : collision_info.second) {
    const double dist_from_bumper =
      std::abs(autoware_utils::inverse_transform_point(poly_vertex.point, bumper_pose).x);

    if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  if (!max_collision_point.has_value() || !max_collision_length.has_value()) return std::nullopt;
  return std::make_pair(
    *max_collision_point,
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, collision_info.first) -
      *max_collision_length);
}

// NOTE: max_lat_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::vector<PointWithStamp> get_collision_points(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_lat_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<PointWithStamp> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obstacle_stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    const auto collision_info = get_collision_index(
      traj_points, traj_polygons, predicted_path.path.at(i), object_time, shape, max_lat_dist);
    if (collision_info) {
      const auto nearest_collision_point = calc_nearest_collision_point(
        collision_info->first, collision_info->second, traj_points, is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(collision_info->first);
    }
  }

  return collision_points;
}

std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length)
{
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware_utils::inverse_transform_pose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};

    // estimate the future ego pose with assuming that the pose error against the reference path
    // will decrease to zero by the time_to_convergence
    if (enable_to_consider_current_pose && time_elapsed < time_to_convergence) {
      const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
      geometry_msgs::msg::Pose indexed_pose_err;
      indexed_pose_err.set__orientation(
        autoware_utils::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
      indexed_pose_err.set__position(
        autoware_utils::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
      current_poses.push_back(
        autoware_utils::transform_pose(indexed_pose_err, traj_points.at(i).pose));
      if (traj_points.at(i).longitudinal_velocity_mps != 0.0) {
        time_elapsed +=
          decimate_trajectory_step_length / std::abs(traj_points.at(i).longitudinal_velocity_mps);
      } else {
        time_elapsed = std::numeric_limits<double>::max();
      }
    }

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        boost::geometry::append(
          idx_poly,
          autoware_utils::to_footprint(pose, front_length, rear_length, vehicle_width).outer());
        boost::geometry::append(
          idx_poly,
          autoware_utils::from_msg(autoware_utils::calc_offset_pose(
                                     pose, front_length, vehicle_width * 0.5 + lat_margin, 0.0)
                                     .position)
            .to_2d());
        boost::geometry::append(
          idx_poly,
          autoware_utils::from_msg(autoware_utils::calc_offset_pose(
                                     pose, front_length, -vehicle_width * 0.5 - lat_margin, 0.0)
                                     .position)
            .to_2d());
      } else {
        boost::geometry::append(
          idx_poly, autoware_utils::to_footprint(
                      pose, front_length, rear_length, vehicle_width + lat_margin * 2.0)
                      .outer());
      }
    }

    boost::geometry::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(tmp_polys, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}
}  // namespace autoware::motion_velocity_planner::polygon_utils
