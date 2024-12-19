// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_CRUISE_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_CRUISE_MODULE_HPP_

#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_stop_module.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_velocity_module_interface.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"
#include "autoware/obstacle_cruise_planner/utils.hpp"

#include <algorithm>
#include <vector>

namespace autoware::motion_planning
{
class ObstacleCruiseModule : public ObstacleVelocityModuleInterface
{
public:
  ObstacleCruiseModule(rclcpp::Node * node, const VehicleInfo & vehicle_info)
  : ObstacleVelocityModuleInterface(node, vehicle_info)
  {
    inside_cruise_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(*node, "common.cruise_obstacle_type.inside.");
    outside_cruise_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(*node, "common.cruise_obstacle_type.outside.");
  }

  std::vector<CruiseObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys, const std::vector<Obstacle> & obstacles,
    const bool is_driving_forward, const BehaviorDeterminationParam & behavior_determination_param)
  {
    is_driving_forward_ = is_driving_forward;
    behavior_determination_param_ = behavior_determination_param;

    // cruise
    std::vector<CruiseObstacle> cruise_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto cruise_obstacle = createCruiseObstacle(
        odometry, decimated_traj_points, decimated_traj_polys, obstacle, obstacle.precise_lat_dist);
      if (cruise_obstacle) {
        cruise_obstacles.push_back(*cruise_obstacle);
      }
    }
    const auto & p = behavior_determination_param_;
    if (p.enable_yield) {
      const auto yield_obstacles = findYieldCruiseObstacles(obstacles, decimated_traj_points);
      if (yield_obstacles) {
        for (const auto & y : yield_obstacles.value()) {
          // Check if there is no member with the same UUID in cruise_obstacles
          auto it = std::find_if(
            cruise_obstacles.begin(), cruise_obstacles.end(),
            [&y](const auto & c) { return y.uuid == c.uuid; });

          // If no matching UUID found, insert yield obstacle into cruise_obstacles
          if (it == cruise_obstacles.end()) {
            cruise_obstacles.push_back(y);
          }
        }
      }
    }
    prev_cruise_object_obstacles_ = cruise_obstacles;

    return cruise_obstacles;
  }

private:
  bool is_driving_forward_;
  std::vector<CruiseObstacle> prev_cruise_object_obstacles_;
  std::vector<int> inside_cruise_obstacle_types_;
  std::vector<int> outside_cruise_obstacle_types_;

  std::optional<CruiseObstacle> createCruiseObstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
    const double precise_lat_dist)
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;

    // NOTE: When driving backward, Stop will be planned instead of cruise.
    //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
    if (!isCruiseObstacle(obstacle.classification.label) || !is_driving_forward_) {
      return std::nullopt;
    }

    if (obstacle.longitudinal_velocity < 0.0) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore obstacle (%s) since it's driving in opposite direction.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (p.max_lat_margin_for_cruise < precise_lat_dist) {
      const auto time_to_traj = precise_lat_dist / std::max(1e-6, obstacle.approach_velocity);
      if (time_to_traj > p.max_lat_time_margin_for_cruise) {
        // RCLCPP_INFO_EXPRESSION(
        //   get_logger(), enable_debug_info_,
        //   "[Cruise] Ignore obstacle (%s) since it's far from trajectory.", object_id.c_str());
        return std::nullopt;
      }
    }

    if (isObstacleCrossing(traj_points, obstacle)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      //   object_id.c_str());
      return std::nullopt;
    }

    const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
      constexpr double epsilon = 1e-6;
      if (precise_lat_dist < epsilon) {
        // obstacle is inside the trajectory
        return createCollisionPointsForInsideCruiseObstacle(traj_points, traj_polys, obstacle);
      }
      // obstacle is outside the trajectory
      // If the ego is stopping, do not plan cruise for outside obstacles. Stop will be planned.
      if (odometry.twist.twist.linear.x < 0.1) {
        return std::nullopt;
      }
      return createCollisionPointsForOutsideCruiseObstacle(traj_points, traj_polys, obstacle);
    }();
    if (!collision_points) {
      return std::nullopt;
    }

    return CruiseObstacle{
      obstacle.uuid,
      obstacle.stamp,
      obstacle.pose,
      obstacle.longitudinal_velocity,
      obstacle.approach_velocity,
      *collision_points};
  }

  std::optional<std::vector<CruiseObstacle>> findYieldCruiseObstacles(
    const std::vector<Obstacle> & obstacles, const std::vector<TrajectoryPoint> & traj_points)
  {
    if (obstacles.empty() || traj_points.empty()) return std::nullopt;
    const auto & p = behavior_determination_param_;

    std::vector<Obstacle> stopped_obstacles;
    std::vector<Obstacle> moving_obstacles;

    std::for_each(
      obstacles.begin(), obstacles.end(),
      [&stopped_obstacles, &moving_obstacles, &p](const auto & o) {
        const bool is_moving =
          std::hypot(o.twist.linear.x, o.twist.linear.y) > p.stopped_obstacle_velocity_threshold;
        if (is_moving) {
          const bool is_within_lat_dist_threshold =
            o.lat_dist_from_obstacle_to_traj < p.yield_lat_distance_threshold;
          if (is_within_lat_dist_threshold) moving_obstacles.push_back(o);
          return;
        }
        // lat threshold is larger for stopped obstacles
        const bool is_within_lat_dist_threshold =
          o.lat_dist_from_obstacle_to_traj <
          p.yield_lat_distance_threshold + p.max_lat_dist_between_obstacles;
        if (is_within_lat_dist_threshold) stopped_obstacles.push_back(o);
        return;
      });

    if (stopped_obstacles.empty() || moving_obstacles.empty()) return std::nullopt;

    std::sort(
      stopped_obstacles.begin(), stopped_obstacles.end(), [](const auto & o1, const auto & o2) {
        return o1.ego_to_obstacle_distance < o2.ego_to_obstacle_distance;
      });

    std::sort(
      moving_obstacles.begin(), moving_obstacles.end(), [](const auto & o1, const auto & o2) {
        return o1.ego_to_obstacle_distance < o2.ego_to_obstacle_distance;
      });

    std::vector<CruiseObstacle> yield_obstacles;
    for (const auto & moving_obstacle : moving_obstacles) {
      for (const auto & stopped_obstacle : stopped_obstacles) {
        const bool is_moving_obs_behind_of_stopped_obs =
          moving_obstacle.ego_to_obstacle_distance < stopped_obstacle.ego_to_obstacle_distance;
        const bool is_moving_obs_ahead_of_ego_front =
          moving_obstacle.ego_to_obstacle_distance > vehicle_info_.vehicle_length_m;

        if (!is_moving_obs_ahead_of_ego_front || !is_moving_obs_behind_of_stopped_obs) continue;

        const double lateral_distance_between_obstacles = std::abs(
          moving_obstacle.lat_dist_from_obstacle_to_traj -
          stopped_obstacle.lat_dist_from_obstacle_to_traj);

        const double longitudinal_distance_between_obstacles = std::abs(
          moving_obstacle.ego_to_obstacle_distance - stopped_obstacle.ego_to_obstacle_distance);

        const double moving_obstacle_speed =
          std::hypot(moving_obstacle.twist.linear.x, moving_obstacle.twist.linear.y);

        const bool are_obstacles_aligned =
          lateral_distance_between_obstacles < p.max_lat_dist_between_obstacles;
        const bool obstacles_collide_within_threshold_time =
          longitudinal_distance_between_obstacles / moving_obstacle_speed <
          p.max_obstacles_collision_time;
        if (are_obstacles_aligned && obstacles_collide_within_threshold_time) {
          const auto yield_obstacle = createYieldCruiseObstacle(moving_obstacle, traj_points);
          if (yield_obstacle) {
            yield_obstacles.push_back(*yield_obstacle);
            using autoware::objects_of_interest_marker_interface::ColorName;
            objects_of_interest_marker_interface_->insertObjectData(
              stopped_obstacle.pose, stopped_obstacle.shape, ColorName::RED);
          }
        }
      }
    }
    if (yield_obstacles.empty()) return std::nullopt;
    return yield_obstacles;
  }

  std::optional<std::vector<PointWithStamp>> createCollisionPointsForInsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle) const
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;

    // check label
    if (!isInsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore inside obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    {  // consider hysteresis
      // const bool is_prev_obstacle_stop = getObstacleFromUuid(prev_stop_obstacles_,
      // obstacle.uuid).has_value();
      const bool is_prev_obstacle_cruise =
        obstacle_cruise_utils::getObstacleFromUuid(prev_cruise_object_obstacles_, obstacle.uuid)
          .has_value();

      if (is_prev_obstacle_cruise) {
        if (obstacle.longitudinal_velocity < p.obstacle_velocity_threshold_from_cruise_to_stop) {
          return std::nullopt;
        }
        // NOTE: else is keeping cruise
      } else {  // if (is_prev_obstacle_stop) {
        // TODO(murooka) consider hysteresis for slow down
        // If previous obstacle is stop or does not exist.
        if (obstacle.longitudinal_velocity < p.obstacle_velocity_threshold_from_stop_to_cruise) {
          return std::nullopt;
        }
        // NOTE: else is cruise from stop
      }
    }

    // Get highest confidence predicted path
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
      obstacle.predicted_paths, p.prediction_resampling_time_interval,
      p.prediction_resampling_time_horizon, 1);

    if (resampled_predicted_paths.empty()) {
      return std::nullopt;
    }

    // calculate nearest collision point
    std::vector<size_t> collision_index;
    const auto collision_points = polygon_utils::getCollisionPoints(
      traj_points, traj_polys, obstacle.stamp, resampled_predicted_paths.front(), obstacle.shape,
      clock_->now(), is_driving_forward_, collision_index,
      obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
        p.decimate_trajectory_step_length +
        std::hypot(
          vehicle_info_.vehicle_length_m,
          vehicle_info_.vehicle_width_m * 0.5 + p.max_lat_margin_for_cruise));
    return collision_points;
  }

  std::optional<std::vector<PointWithStamp>> createCollisionPointsForOutsideCruiseObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polys,
    const Obstacle & obstacle) const
  {
    const auto & p = behavior_determination_param_;
    const auto & object_id = obstacle.uuid.substr(0, 4);

    // check label
    if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (
      std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
      p.outside_obstacle_min_velocity_threshold) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      //   object_id.c_str());
      return std::nullopt;
    }

    // Get the highest confidence predicted paths
    const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
      obstacle.predicted_paths, p.prediction_resampling_time_interval,
      p.prediction_resampling_time_horizon, p.num_of_predicted_paths_for_outside_cruise_obstacle);

    // calculate collision condition for cruise
    std::vector<size_t> collision_index;
    const auto getCollisionPoints = [&]() -> std::vector<PointWithStamp> {
      for (const auto & predicted_path : resampled_predicted_paths) {
        const auto collision_points = polygon_utils::getCollisionPoints(
          traj_points, traj_polys, obstacle.stamp, predicted_path, obstacle.shape, clock_->now(),
          is_driving_forward_, collision_index,
          obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
            p.decimate_trajectory_step_length +
            std::hypot(
              vehicle_info_.vehicle_length_m,
              vehicle_info_.vehicle_width_m * 0.5 + p.max_lat_margin_for_cruise),
          p.max_prediction_time_for_collision_check);
        if (!collision_points.empty()) {
          return collision_points;
        }
      }
      return {};
    };

    const auto collision_points = getCollisionPoints();

    if (collision_points.empty()) {
      // Ignore vehicle obstacles outside the trajectory without collision
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    const double overlap_time =
      (rclcpp::Time(collision_points.back().stamp) - rclcpp::Time(collision_points.front().stamp))
        .seconds();
    if (overlap_time < p.ego_obstacle_overlap_time_threshold) {
      // Ignore vehicle obstacles outside the trajectory, whose predicted path
      // overlaps the ego trajectory in a certain time.
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    // Ignore obstacles behind the ego vehicle.
    // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
    // properly when the trajectory is crossing or overlapping.
    const size_t first_collision_index = collision_index.front();
    if (!isFrontCollideObstacle(traj_points, obstacle, first_collision_index)) {
      return std::nullopt;
    }
    return collision_points;
  }

  std::optional<CruiseObstacle> createYieldCruiseObstacle(
    const Obstacle & obstacle, const std::vector<TrajectoryPoint> & traj_points)
  {
    if (traj_points.empty()) return std::nullopt;
    // check label
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;

    if (!isOutsideCruiseObstacle(obstacle.classification.label)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since its type is not designated.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (
      std::hypot(obstacle.twist.linear.x, obstacle.twist.linear.y) <
      p.outside_obstacle_min_velocity_threshold) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since the obstacle velocity is low.",
      //   object_id.c_str());
      return std::nullopt;
    }

    if (isObstacleCrossing(traj_points, obstacle)) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Cruise] Ignore yield obstacle (%s) since it's crossing the ego's trajectory..",
      //   object_id.c_str());
      return std::nullopt;
    }

    const auto collision_points = [&]() -> std::optional<std::vector<PointWithStamp>> {
      const auto obstacle_idx =
        autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose);
      if (!obstacle_idx) return std::nullopt;
      const auto collision_traj_point = traj_points.at(obstacle_idx.value());
      const auto object_time = clock_->now() + traj_points.at(obstacle_idx.value()).time_from_start;

      PointWithStamp collision_traj_point_with_stamp;
      collision_traj_point_with_stamp.stamp = object_time;
      collision_traj_point_with_stamp.point.x = collision_traj_point.pose.position.x;
      collision_traj_point_with_stamp.point.y = collision_traj_point.pose.position.y;
      collision_traj_point_with_stamp.point.z = collision_traj_point.pose.position.z;
      std::vector<PointWithStamp> collision_points_vector{collision_traj_point_with_stamp};
      return collision_points_vector;
    }();

    if (!collision_points) return std::nullopt;
    // check if obstacle is driving on the opposite direction
    if (obstacle.longitudinal_velocity < 0.0) return std::nullopt;
    return CruiseObstacle{
      obstacle.uuid,
      obstacle.stamp,
      obstacle.pose,
      obstacle.longitudinal_velocity,
      obstacle.approach_velocity,
      collision_points.value(),
      true};
  }

  bool isInsideCruiseObstacle(const uint8_t label) const
  {
    const auto & types = inside_cruise_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isOutsideCruiseObstacle(const uint8_t label) const
  {
    const auto & types = outside_cruise_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isCruiseObstacle(const uint8_t label) const
  {
    return isInsideCruiseObstacle(label) || isOutsideCruiseObstacle(label);
  }

  bool isFrontCollideObstacle(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle,
    const size_t first_collision_idx) const
  {
    const auto obstacle_idx =
      autoware::motion_utils::findNearestIndex(traj_points, obstacle.pose.position);

    const double obstacle_to_col_points_distance =
      autoware::motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
    const double obstacle_max_length = obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape);

    // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
    return obstacle_to_col_points_distance > -obstacle_max_length;
  }

  bool isObstacleCrossing(
    const std::vector<TrajectoryPoint> & traj_points, const Obstacle & obstacle) const
  {
    const double diff_angle = calcDiffAngleAgainstTrajectory(traj_points, obstacle.pose);

    // NOTE: Currently predicted objects does not have orientation availability even
    // though sometimes orientation is not available.
    const bool is_obstacle_crossing_trajectory =
      behavior_determination_param_.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
      behavior_determination_param_.crossing_obstacle_traj_angle_threshold <
        M_PI - std::abs(diff_angle);
    if (!is_obstacle_crossing_trajectory) {
      return false;
    }

    // Only obstacles crossing the ego's trajectory with high speed are considered.
    return true;
  }
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_CRUISE_MODULE_HPP_
