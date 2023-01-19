// Copyright 2023 TIER IV, Inc.
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

#include "collision_free_path_planner/utils/geometry_utils.hpp"

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

namespace collision_free_path_planner
{
namespace
{
geometry_msgs::msg::Point getStartPoint(
  const std::vector<geometry_msgs::msg::Point> & bound, const geometry_msgs::msg::Point & point)
{
  const size_t segment_idx = motion_utils::findNearestSegmentIndex(bound, point);
  const auto & curr_seg_point = bound.at(segment_idx);
  const auto & next_seg_point = bound.at(segment_idx);
  const Eigen::Vector2d first_to_target{point.x - curr_seg_point.x, point.y - curr_seg_point.y};
  const Eigen::Vector2d first_to_second{
    next_seg_point.x - curr_seg_point.x, next_seg_point.y - curr_seg_point.y};
  const double length = first_to_target.dot(first_to_second.normalized());

  if (length < 0.0) {
    return bound.front();
  }

  const auto first_point = motion_utils::calcLongitudinalOffsetPoint(bound, segment_idx, length);

  if (first_point) {
    return *first_point;
  }

  return bound.front();
}

bool isOutsideDrivableArea(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  if (left_bound.empty() || right_bound.empty()) {
    return false;
  }

  constexpr double min_dist = 0.1;
  const auto left_start_point = getStartPoint(left_bound, right_bound.front());
  const auto right_start_point = getStartPoint(right_bound, left_bound.front());

  // ignore point in front of the front line
  const std::vector<geometry_msgs::msg::Point> front_bound = {left_start_point, right_start_point};
  const double lat_dist_to_front_bound = motion_utils::calcLateralOffset(front_bound, point);
  if (lat_dist_to_front_bound > min_dist) {
    return false;
  }

  // left bound check
  const double lat_dist_to_left_bound = motion_utils::calcLateralOffset(left_bound, point);
  if (lat_dist_to_left_bound > min_dist) {
    return true;
  }

  // right bound check
  const double lat_dist_to_right_bound = motion_utils::calcLateralOffset(right_bound, point);
  if (lat_dist_to_right_bound < -min_dist) {
    return true;
  }

  return false;
}
}  // namespace

namespace geometry_utils
{
bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  if (left_bound.empty() || right_bound.empty()) {
    return false;
  }

  const double offset_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
  const double offset_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
  const double offset_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double offset_rear = vehicle_info.rear_overhang_m;

  // define function to check if the offsetted point is outside the bounds
  const auto is_offsetted_point_outside = [&](const double offset_x, const double offset_y) {
    const auto offset_pos =
      tier4_autoware_utils::calcOffsetPose(pose, offset_x, offset_y, 0.0).position;
    return isOutsideDrivableArea(offset_pos, left_bound, right_bound);
  };

  const bool is_top_left_outside = is_offsetted_point_outside(offset_front, -offset_left);
  const bool is_top_right_outside = is_offsetted_point_outside(offset_front, offset_right);
  const bool is_bottom_left_outside = is_offsetted_point_outside(-offset_rear, -offset_left);
  const bool is_bottom_right_outside = is_offsetted_point_outside(-offset_rear, offset_right);

  if (
    is_top_left_outside || is_top_right_outside || is_bottom_left_outside ||
    is_bottom_right_outside) {
    return true;
  }

  return false;
}
}  // namespace geometry_utils
}  // namespace collision_free_path_planner
