// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TTC_UTILS_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TTC_UTILS_HPP_

#include "collision_checker.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct CollisionTimeRange
{
  double from_s;  // [s] time at the start of the collision
  double to_s;    // [s] time at the end of the collision

  /// @brief constructor from a single time point
  /// @param t [s] time point
  explicit CollisionTimeRange(const double t) : from_s(t), to_s(t) {}

  /// @brief constructor from a start and end time points
  /// @param from [s] start time point
  /// @param to [s] end time point
  CollisionTimeRange(const double from, const double to) : from_s(from), to_s(to)
  {
    if (from > to) {
      throw std::invalid_argument("Start time must be less than or equal to end time.");
    }
  }

  [[nodiscard]] double time_to_collision(const CollisionTimeRange & other) const
  {
    // before the other range
    if (to_s < other.from_s) {
      return other.from_s - to_s;
    }
    // after the other range
    if (other.to_s < from_s) {
      return from_s - other.to_s;
    }
    // overlapping
    return 0.0;
  }

  void extend(const double t)
  {
    if (t < from_s) {
      from_s = t;
    }
    if (t > to_s) {
      to_s = t;
    }
  }

  [[nodiscard]] std::string to_string() const
  {
    return std::to_string(from_s) + "-" + std::to_string(to_s);
  }
};

using TimeCollisions = std::map<double, universe_utils::MultiPoint2d>;

using CollisionTimeRanges = std::vector<std::optional<CollisionTimeRange>>;

/// @brief calculate the time distance of a predicted object at each ego trajectory point
/// @param ego_collision_checker
/// @param predicted_object
/// @return for each ego trajectory point, the time ranges when the object will collide with ego
CollisionTimeRanges calculate_object_to_trajectory_time_ranges(
  const CollisionChecker & ego_collision_checker,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object);

/// @brief calculate time to collisions for each ego trajectory point
/// @param ego_collision_checker collision checker for the ego trajectory
/// @param ego_trajectory ego trajectory with accurate time_from_start values
/// @param predicted_objects predicted object to check for collisions
/// @return for each ego trajectory point, the ttc ranges when the object will collide with ego
std::vector<std::optional<CollisionTimeRange>> calculate_collision_time_ranges_along_trajectory(
  const CollisionChecker & ego_collision_checker,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object);

/// @brief calculate, for each ego trajectory point, when and where the object intersect with the
/// ego footprint at the point
/// @param ego_collision_checker collision checker for the ego trajectory
/// @param predicted_objects predicted object to check for collisions
/// @return for each ego trajectory point, the collision points for each time step with collision
std::vector<TimeCollisions> calculate_time_collisions_along_trajectory(
  const CollisionChecker & ego_collision_checker,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object);

}  // namespace autoware::motion_velocity_planner
#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TTC_UTILS_HPP_
