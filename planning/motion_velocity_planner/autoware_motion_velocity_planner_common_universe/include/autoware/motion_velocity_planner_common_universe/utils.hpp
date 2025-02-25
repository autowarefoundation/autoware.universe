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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__UTILS_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__UTILS_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "planner_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::Polygon2d;
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

geometry_msgs::msg::Point to_geometry_point(const pcl::PointXYZ & point);
geometry_msgs::msg::Point to_geometry_point(const autoware_utils::Point2d & point);

std::optional<double> calc_distance_to_front_object(
  const std::vector<TrajectoryPoint> & traj_points, const size_t ego_idx,
  const geometry_msgs::msg::Point & obstacle_pos);

template <class T>
std::vector<T> concat_vectors(std::vector<T> first_vector, std::vector<T> second_vector)
{
  first_vector.insert(
    first_vector.end(), std::make_move_iterator(second_vector.begin()),
    std::make_move_iterator(second_vector.end()));
  return first_vector;
}

std::vector<TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length, const double goal_extended_trajectory_length);

template <typename T>
std::optional<T> get_obstacle_from_uuid(
  const std::vector<T> & obstacles, const std::string & target_uuid)
{
  const auto itr = std::find_if(obstacles.begin(), obstacles.end(), [&](const auto & obstacle) {
    return obstacle.uuid == target_uuid;
  });

  if (itr == obstacles.end()) {
    return std::nullopt;
  }
  return *itr;
}

std::vector<uint8_t> get_target_object_type(rclcpp::Node & node, const std::string & param_prefix);

double calc_object_possible_max_dist_from_center(const Shape & shape);

Marker get_object_marker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b);

template <class T>
size_t get_index_with_longitudinal_offset(
  const T & points, const double longitudinal_offset, std::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= *start_idx) {
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
    for (size_t i = *start_idx; i < points.size() - 1; ++i) {
      const double segment_length = autoware_utils::calc_distance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double back_length = sum_length - longitudinal_offset;
        const double front_length = segment_length - back_length;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = *start_idx; 0 < i; --i) {
    const double segment_length = autoware_utils::calc_distance2d(points.at(i - 1), points.at(i));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double back_length = sum_length + longitudinal_offset;
      const double front_length = segment_length - back_length;
      if (front_length < back_length) {
        return i;
      } else {
        return i - 1;
      }
    }
  }
  return 0;
}

double calc_possible_min_dist_from_obj_to_traj_poly(
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info);
}  // namespace autoware::motion_velocity_planner::utils

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON_UNIVERSE__UTILS_HPP_
