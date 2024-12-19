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

#ifndef AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_
#define AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_

#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware/obstacle_cruise_planner/common_structs.hpp"
#include "autoware/obstacle_cruise_planner/obstacle_velocity_module_interface.hpp"
#include "autoware/obstacle_cruise_planner/polygon_utils.hpp"
#include "autoware/obstacle_cruise_planner/type_alias.hpp"

#include <algorithm>
#include <vector>

namespace autoware::motion_planning
{
namespace
{
double calcDiffAngleAgainstTrajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & target_pose)
{
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(traj_points, target_pose.position);
  const double traj_yaw = tf2::getYaw(traj_points.at(nearest_idx).pose.orientation);

  const double target_yaw = tf2::getYaw(target_pose.orientation);

  const double diff_yaw = autoware::universe_utils::normalizeRadian(target_yaw - traj_yaw);
  return diff_yaw;
}
}  // namespace

std::vector<PredictedPath> resampleHighestConfidencePredictedPaths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

class ObstacleStopModule : public ObstacleVelocityModuleInterface
{
public:
  ObstacleStopModule(rclcpp::Node * node, const VehicleInfo & vehicle_info)
  : ObstacleVelocityModuleInterface(node, vehicle_info)
  {
    inside_stop_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(*node, "common.stop_obstacle_type.inside.");
    outside_stop_obstacle_types_ =
      obstacle_cruise_utils::getTargetObjectType(*node, "common.stop_obstacle_type.outside.");
    use_pointcloud_for_stop_ =
      node->declare_parameter<bool>("common.stop_obstacle_type.pointcloud");
  }

  std::vector<StopObstacle> determineEgoBehaviorAgainstPredictedObjectObstacles(
    const Odometry & odometry, const PredictedObjects & objects,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<Polygon2d> & decimated_traj_polys, const std::vector<Obstacle> & obstacles,
    const bool is_driving_forward, const BehaviorDeterminationParam & behavior_determination_param,
    const double min_behavior_stop_margin)
  {
    is_driving_forward_ = is_driving_forward;
    behavior_determination_param_ = behavior_determination_param;
    min_behavior_stop_margin_ = min_behavior_stop_margin;

    // stop
    std::vector<StopObstacle> stop_obstacles;
    for (const auto & obstacle : obstacles) {
      const auto stop_obstacle = createStopObstacleForPredictedObject(
        odometry, decimated_traj_points, decimated_traj_polys, obstacle, obstacle.precise_lat_dist);
      if (stop_obstacle) {
        stop_obstacles.push_back(*stop_obstacle);
      }
    }
    // Check target obstacles' consistency
    checkConsistency(objects.header.stamp, objects, stop_obstacles);
    prev_stop_object_obstacles_ = stop_obstacles;

    return stop_obstacles;
  }

  double calcTimeToReachCollisionPoint(
    const Odometry & odometry, const geometry_msgs::msg::Point & collision_point,
    const std::vector<TrajectoryPoint> & traj_points, const double abs_ego_offset) const
  {
    const auto & p = behavior_determination_param_;
    const double dist_from_ego_to_obstacle =
      std::abs(autoware::motion_utils::calcSignedArcLength(
        traj_points, odometry.pose.pose.position, collision_point)) -
      abs_ego_offset;
    return dist_from_ego_to_obstacle /
           std::max(
             p.min_velocity_to_reach_collision_point, std::abs(odometry.twist.twist.linear.x));
  }

private:
  double min_behavior_stop_margin_;
  std::vector<StopObstacle> prev_closest_stop_object_obstacles_{};
  bool use_pointcloud_for_stop_;
  bool is_driving_forward_;
  std::vector<StopObstacle> prev_stop_object_obstacles_;
  std::vector<int> inside_stop_obstacle_types_;
  std::vector<int> outside_stop_obstacle_types_;

  std::optional<StopObstacle> createStopObstacleForPredictedObject(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
    const double precise_lat_dist) const
  {
    const auto & p = behavior_determination_param_;
    const auto & object_id = obstacle.uuid.substr(0, 4);

    if (!isStopObstacle(obstacle.classification.label)) {
      return std::nullopt;
    }
    const double max_lat_margin_for_stop =
      (obstacle.classification.label == ObjectClassification::UNKNOWN)
        ? p.max_lat_margin_for_stop_against_unknown
        : p.max_lat_margin_for_stop;

    // Obstacle that is not inside of trajectory
    if (precise_lat_dist > std::max(max_lat_margin_for_stop, 1e-3)) {
      if (!isOutsideStopObstacle(obstacle.classification.label)) {
        return std::nullopt;
      }

      const auto time_to_traj = precise_lat_dist / std::max(1e-6, obstacle.approach_velocity);
      if (time_to_traj > p.max_lat_time_margin_for_stop) {
        // RCLCPP_INFO_EXPRESSION(
        //   get_logger(), enable_debug_info_,
        //   "[Stop] Ignore outside obstacle (%s) since it's far from trajectory.",
        //   object_id.c_str());
        return std::nullopt;
      }

      // brkay54: For the pedestrians and bicycles, we need to check the collision point by thinking
      // they will stop with a predefined deceleration rate to avoid unnecessary stops.
      double resample_time_horizon = p.prediction_resampling_time_horizon;
      if (obstacle.classification.label == ObjectClassification::PEDESTRIAN) {
        resample_time_horizon =
          std::sqrt(std::pow(obstacle.twist.linear.x, 2) + std::pow(obstacle.twist.linear.y, 2)) /
          (2.0 * p.pedestrian_deceleration_rate);
      } else if (obstacle.classification.label == ObjectClassification::BICYCLE) {
        resample_time_horizon =
          std::sqrt(std::pow(obstacle.twist.linear.x, 2) + std::pow(obstacle.twist.linear.y, 2)) /
          (2.0 * p.bicycle_deceleration_rate);
      }

      // Get the highest confidence predicted path
      const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
        obstacle.predicted_paths, p.prediction_resampling_time_interval, resample_time_horizon,
        p.num_of_predicted_paths_for_outside_stop_obstacle);
      if (resampled_predicted_paths.empty()) {
        return std::nullopt;
      }

      const auto getCollisionPoint =
        [&]() -> std::optional<std::pair<geometry_msgs::msg::Point, double>> {
        for (const auto & predicted_path : resampled_predicted_paths) {
          const auto collision_point = createCollisionPointForOutsideStopObstacle(
            odometry, traj_points, traj_polys, obstacle, predicted_path, max_lat_margin_for_stop);
          if (collision_point) {
            return collision_point;
          }
        }
        return std::nullopt;
      };

      const auto collision_point = getCollisionPoint();

      if (collision_point) {
        return StopObstacle{
          obstacle.uuid,
          obstacle.stamp,
          obstacle.classification,
          obstacle.pose,
          obstacle.shape,
          obstacle.longitudinal_velocity,
          obstacle.approach_velocity,
          collision_point->first,
          collision_point->second};
      }
      return std::nullopt;
    }

    // Obstacle inside the trajectory
    if (!isInsideStopObstacle(obstacle.classification.label)) {
      return std::nullopt;
    }

    // calculate collision points with trajectory with lateral stop margin
    const auto traj_polys_with_lat_margin = obstacle_cruise_utils::createOneStepPolygons(
      traj_points, vehicle_info_, odometry.pose.pose, max_lat_margin_for_stop,
      behavior_determination_param_);

    const auto collision_point = polygon_utils::getCollisionPoint(
      traj_points, traj_polys_with_lat_margin, obstacle, is_driving_forward_, vehicle_info_);
    if (!collision_point) {
      return std::nullopt;
    }

    // check transient obstacle or not
    const double abs_ego_offset =
      min_behavior_stop_margin_ + (is_driving_forward_
                                     ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                     : std::abs(vehicle_info_.min_longitudinal_offset_m));

    const double time_to_reach_stop_point =
      calcTimeToReachCollisionPoint(odometry, collision_point->first, traj_points, abs_ego_offset);
    const bool is_transient_obstacle = [&]() {
      if (time_to_reach_stop_point <= p.collision_time_margin) {
        return false;
      }
      // get the predicted position of the obstacle when ego reaches the collision point
      const auto resampled_predicted_paths = resampleHighestConfidencePredictedPaths(
        obstacle.predicted_paths, p.prediction_resampling_time_interval,
        p.prediction_resampling_time_horizon, 1);
      if (resampled_predicted_paths.empty() || resampled_predicted_paths.front().path.empty()) {
        return false;
      }
      const auto future_obj_pose = autoware::object_recognition_utils::calcInterpolatedPose(
        resampled_predicted_paths.front(), time_to_reach_stop_point - p.collision_time_margin);

      Obstacle tmp_future_obs = obstacle;
      tmp_future_obs.pose =
        future_obj_pose ? future_obj_pose.value() : resampled_predicted_paths.front().path.back();
      const auto future_collision_point = polygon_utils::getCollisionPoint(
        traj_points, traj_polys_with_lat_margin, tmp_future_obs, is_driving_forward_,
        vehicle_info_);

      return !future_collision_point;
    }();

    if (is_transient_obstacle) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore inside obstacle (%s) since it is transient obstacle.", object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    return StopObstacle{
      obstacle.uuid,
      obstacle.stamp,
      obstacle.classification,
      obstacle.pose,
      obstacle.shape,
      obstacle.longitudinal_velocity,
      obstacle.approach_velocity,
      collision_point->first,
      collision_point->second};
  }

  bool isInsideStopObstacle(const uint8_t label) const
  {
    const auto & types = inside_stop_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isOutsideStopObstacle(const uint8_t label) const
  {
    const auto & types = outside_stop_obstacle_types_;
    return std::find(types.begin(), types.end(), label) != types.end();
  }

  bool isStopObstacle(const uint8_t label) const
  {
    return isInsideStopObstacle(label) || isOutsideStopObstacle(label);
  }

  void checkConsistency(
    const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
    std::vector<StopObstacle> & stop_obstacles)
  {
    for (const auto & prev_closest_stop_obstacle : prev_closest_stop_object_obstacles_) {
      const auto predicted_object_itr = std::find_if(
        predicted_objects.objects.begin(), predicted_objects.objects.end(),
        [&prev_closest_stop_obstacle](const PredictedObject & po) {
          return autoware::universe_utils::toHexString(po.object_id) ==
                 prev_closest_stop_obstacle.uuid;
        });
      // If previous closest obstacle disappear from the perception result, do nothing anymore.
      if (predicted_object_itr == predicted_objects.objects.end()) {
        continue;
      }

      const auto is_disappeared_from_stop_obstacle = std::none_of(
        stop_obstacles.begin(), stop_obstacles.end(),
        [&prev_closest_stop_obstacle](const StopObstacle & so) {
          return so.uuid == prev_closest_stop_obstacle.uuid;
        });
      if (is_disappeared_from_stop_obstacle) {
        // re-evaluate as a stop candidate, and overwrite the current decision if "maintain stop"
        // condition is satisfied
        const double elapsed_time = (current_time - prev_closest_stop_obstacle.stamp).seconds();
        if (
          predicted_object_itr->kinematics.initial_twist_with_covariance.twist.linear.x <
            behavior_determination_param_.obstacle_velocity_threshold_from_stop_to_cruise &&
          elapsed_time < behavior_determination_param_.stop_obstacle_hold_time_threshold) {
          stop_obstacles.push_back(prev_closest_stop_obstacle);
        }
      }
    }

    prev_closest_stop_object_obstacles_ =
      obstacle_cruise_utils::getClosestStopObstacles(stop_obstacles);
  }

  std::optional<std::pair<geometry_msgs::msg::Point, double>>
  createCollisionPointForOutsideStopObstacle(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<Polygon2d> & traj_polys, const Obstacle & obstacle,
    const PredictedPath & resampled_predicted_path, double max_lat_margin_for_stop) const
  {
    const auto & object_id = obstacle.uuid.substr(0, 4);
    const auto & p = behavior_determination_param_;

    std::vector<size_t> collision_index;
    const auto collision_points = polygon_utils::getCollisionPoints(
      traj_points, traj_polys, obstacle.stamp, resampled_predicted_path, obstacle.shape,
      clock_->now(), is_driving_forward_, collision_index,
      obstacle_cruise_utils::calcObstacleMaxLength(obstacle.shape) +
        p.decimate_trajectory_step_length +
        std::hypot(
          vehicle_info_.vehicle_length_m,
          vehicle_info_.vehicle_width_m * 0.5 + max_lat_margin_for_stop));
    if (collision_points.empty()) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore outside obstacle (%s) since there is no collision point between the "
      //   "predicted path "
      //   "and the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    const double collision_time_margin =
      calcCollisionTimeMargin(odometry, collision_points, traj_points, is_driving_forward_);
    if (p.collision_time_margin < collision_time_margin) {
      // RCLCPP_INFO_EXPRESSION(
      //   get_logger(), enable_debug_info_,
      //   "[Stop] Ignore outside obstacle (%s) since it will not collide with the ego.",
      //   object_id.c_str());
      // debug_data_ptr_->intentionally_ignored_obstacles.push_back(obstacle);
      return std::nullopt;
    }

    return polygon_utils::getCollisionPoint(
      traj_points, collision_index.front(), collision_points, is_driving_forward_, vehicle_info_);
  }
  double calcCollisionTimeMargin(
    const Odometry & odometry, const std::vector<PointWithStamp> & collision_points,
    const std::vector<TrajectoryPoint> & traj_points, const bool is_driving_forward) const
  {
    const auto & p = behavior_determination_param_;
    const double abs_ego_offset =
      min_behavior_stop_margin_ + (is_driving_forward
                                     ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                     : std::abs(vehicle_info_.min_longitudinal_offset_m));
    const double time_to_reach_stop_point = ObstacleStopModule::calcTimeToReachCollisionPoint(
      odometry, collision_points.front().point, traj_points, abs_ego_offset);

    const double time_to_leave_collision_point =
      time_to_reach_stop_point +
      abs_ego_offset /
        std::max(p.min_velocity_to_reach_collision_point, odometry.twist.twist.linear.x);

    const double time_to_start_cross =
      (rclcpp::Time(collision_points.front().stamp) - clock_->now()).seconds();
    const double time_to_end_cross =
      (rclcpp::Time(collision_points.back().stamp) - clock_->now()).seconds();

    if (time_to_leave_collision_point < time_to_start_cross) {  // Ego goes first.
      return time_to_start_cross - time_to_reach_stop_point;
    }
    if (time_to_end_cross < time_to_reach_stop_point) {  // Obstacle goes first.
      return time_to_reach_stop_point - time_to_end_cross;
    }
    return 0.0;  // Ego and obstacle will collide.
  }
};
}  // namespace autoware::motion_planning

#endif  // AUTOWARE__OBSTACLE_CRUISE_PLANNER__OBSTACLE_STOP_MODULE_HPP_
